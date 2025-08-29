/**
 * @file   dwc_queue.h
 *
 * @brief  DWC queue implementation
 */
/*
 *  Copyright (C) 2025 Texas Instruments Incorporated
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
/** ============================================================================*/

#ifndef _DWC_QUEUE_NORTOS_H_
#define _DWC_QUEUE_NORTOS_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DWC_QUEUE_SIZE     (8U)

/**
 * @brief A structure representing a queue for managing data.
 *
 * This structure is used to implement a queue with a fixed-size buffer.
 */
typedef struct {
    uint32_t buffer[DWC_QUEUE_SIZE]; /**< An array of size DWC_QUEUE_SIZE used to store the queue's data. */
    uint16_t front; /**< The index of the front element in the queue. */
    uint16_t rear;  /**< The index of the rear element in the queue. */
}dwc_queue_t;

void dwc_queueInit(dwc_queue_t *queue);
bool dwc_queuePut(dwc_queue_t *queue, uint32_t data);
bool dwc_queueGet(dwc_queue_t *queue, uint32_t *data);


#ifdef __cplusplus
 }
#endif

#endif /* _DWC_QUEUE_NORTOS_H_ */
