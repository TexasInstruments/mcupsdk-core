/*
 *  Copyright (C) 2018-2023 Texas Instruments Incorporated
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

#ifndef CYCLE_COUNTERP_H
#define CYCLE_COUNTERP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <kernel/dpl/SystemP.h>

/**
 * \defgroup KERNEL_DPL_CYCLE_COUNTER APIs for Counting CPU Cycles
 * \ingroup KERNEL_DPL
 *
 * For more details and example usage, see \ref KERNEL_DPL_CYCLE_COUNTER_PAGE
 *
 * @{
 */

/**
 * \brief initialize PMU Cycle Counter
 *
 */
void CycleCounterP_init(const uint64_t cpuFreqHz);

/**
 * \brief Get 32b CPU cycle counter value
 *
 * Make sure to handle overflow condition in your application.
 *
 * \return 32b cycle counter value
 */
uint32_t CycleCounterP_getCount32(void);

/**
 * \brief Enable, reset, clear overflow for CPU cycle counter
 *
 * - Call this API atleast once before using CycleCounterP_getCount32() to reset and enable the counter
 * - Call this API to reset counter to zero.
 */
void CycleCounterP_reset(void);

/**
 * \brief Get 64b CPU cycle counter value
 *
 * Only support with below CPUs,
 * - A53
 *
 * \return 64b cycle counter value
 */
uint64_t CycleCounterP_getCount64(void);

/**
 * \brief API function to convert nanosecs to PMU counter ticks
 *
 * \param nanosecs    time unit in nano sec
 *
 *\return PMU counter ticks
 */
uint64_t CycleCounterP_nsToTicks(const uint64_t nanosecs);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* MPUP_ARM_V7_H */
