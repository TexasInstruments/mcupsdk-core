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

#ifndef SYSTICKTIMERP_H
#define SYSTICKTIMERP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/TimerP.h>

/*
 * APIs for sys tick timer on M4
 */

/**
 * \brief Set default value for \ref TimerP_Params
 * 
 * \param params [out] timer parameters initalized to default
 */ 
void SysTickTimerP_Params_init(TimerP_Params *params);

/**
 * \brief Setup timer but does not start it
 * 
 * \param params [in] timer parameters
 */ 
void SysTickTimerP_setup(TimerP_Params *params);

/**
 * \brief Start timer 
 */ 
void SysTickTimerP_start();

/**
 * \brief Stop timer
 */ 
void SysTickTimerP_stop();

/**
 * \brief Get timer current count
 * 
 * \return current timer count value
 */ 
uint32_t SysTickTimerP_getCount();

/**
 * \brief Get timer reload count
 * 
 * \return reload count value
 */ 
uint32_t SysTickTimerP_getReloadCount();

/**
 * \brief Check if timer is overflowed
 */ 
uint32_t SysTickTimerP_isOverflowed();

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* TIMERP_H */

