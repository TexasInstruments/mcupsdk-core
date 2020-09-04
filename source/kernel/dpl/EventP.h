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

#ifndef EVENTP_H
#define EVENTP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <kernel/dpl/SystemP.h>

/**
 * \defgroup KERNEL_DPL_EVENT APIs for Event
 * \ingroup KERNEL_DPL
 *
 * For more details and example usage, see \ref KERNEL_DPL_EVENT_PAGE
 *
 * @{
 */

/**
 * \brief Max size of Event object across no-RTOS and all OS's
 */
#define EventP_OBJECT_SIZE_MAX    (60u)
/**
 * \brief Opaque Event object used with the Event APIs
 */
typedef struct EventP_Object_
{
    /* uintptr_t translates to uint64_t for A53 and uint32_t for R5 and M4 */
    /* This accounts for the 64bit pointer in A53 and 32bit pointer in R5 and M4 */
    uintptr_t rsv[EventP_OBJECT_SIZE_MAX/sizeof(uint32_t)]; /**< reserved, should NOT be modified by end users */
} EventP_Object;

/**
 * \brief Create an Event object
 *
 * \param obj [out] Created Event object
 *
 * \return \ref SystemP_SUCCESS on success, \ref SystemP_FAILURE on error
 */
int32_t EventP_construct(EventP_Object *obj);

/**
 * \brief Cleanup, delete, destruct an Event object
 *
 * \param obj [in] Event object
 */
void EventP_destruct(EventP_Object *obj);

/**
 * \brief Read the Event bits, after optionally waiting for a bit or multiple bits
 *        to be set
 *
 * \param obj               [in] Event object
 * \param bitsToWaitFor     [in] Bitwise value indicating the bits to be tested
 * \param clearOnExit       [in] If this parameter is 1, then any bits set in the
 *                               value passed in bitsToWaitFor will be cleared
 *                               before this API returns. If there is a time-out
 *                               based on timeToWaitInTicks parameter,
 *                               it will not be cleared. \n
 *                               If this parameter is 0, then none of the bits
 *                               are altered.
 * \param waitForAll        [in] This parameter is used to specify if the test of
 *                               bits should be logical AND or logical OR. \n
 *                               If this parameter is 1, then this API will return
 *                               when ALL the bits set in bitsToWaitFor are
 *                               set in the Event bits, or if there is a time-out. \n
 *                               If this parameter is 0, then this API will return
 *                               when ANY of the bits set in bitsToWaitFor is
 *                               set in the Event bits, or if there is a time-out. \n
 * \param timeToWaitInTicks [in] Amount of time to wait for one or all bits from
 *                               bitsToWaitFor to be set, in units of system ticks
 *                               (see \ref KERNEL_DPL_CLOCK_PAGE)
 * \param eventBits         [out]Value of the Event bits when the bits being
 *                               waited for were set, or the time-out occured.
 *
 *
 * \return \ref SystemP_SUCCESS on success or time-out
 * \return \ref SystemP_FAILURE on failure.
 *
 * \note This function should not be called from an interrupt.
 *
 * \note The actual current value of Event bits can be different from returned
 *       value if an interrupt or a higher priority task modifies the value between
 *       the calling task leaving blocked state and exiting this API.
 */
int32_t EventP_waitBits(EventP_Object  *obj,
                        uint32_t       bitsToWaitFor,
                        uint8_t        clearOnExit,
                        uint8_t        waitForAll,
                        uint32_t       timeToWaitInTicks,
                        uint32_t       *eventBits);

/**
 * \brief Setting a bit or multiple bits in the Event bits
 *
 * \param obj               [in] Event object
 * \param bitsToSet         [in] Bitwise value indicating the bits to be set
 *
 * \return \ref SystemP_SUCCESS on success
 * \return \ref SystemP_FAILURE on failure
 *

 */
int32_t EventP_setBits(EventP_Object *obj, uint32_t bitsToSet);

/**
 * \brief Clear a bit or multiple bits in the Event bits
 *
 * \param obj               [in] Event object
 * \param bitsToClear       [in] Bitwise value indicating the bits to be cleared
 *
 * \return \ref SystemP_SUCCESS on success
 * \return \ref SystemP_FAILURE on failure
 *
 */
int32_t EventP_clearBits(EventP_Object *obj, uint32_t bitsToClear);

/**
 * \brief Getting the current value of Event bits
 *
 * \param obj               [in] Event object
 * \param eventBits         [out]Value of the Event bits when the bits being
 *                               waited for were set, or the time-out occured.
 *
 * \return \ref SystemP_SUCCESS on success.
 * \return \ref SystemP_FAILURE on failure.
 *
 * \note The actual current value of Event bits can be different from returned
 *       value if an interrupt or a higher priority task modifies the value between
 *       the calling task leaving blocked state and exiting this API.
 */
int32_t EventP_getBits(EventP_Object *obj, uint32_t *eventBits);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* EVENTP_H */
