/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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

#ifndef BOOTLOADER_CAN_H_
#define BOOTLOADER_CAN_H_

#ifdef __cplusplus
extern "C"
{
#endif


#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define CONFIG_MCAN0 (0U)
#define ENABLE_CANFD_SUPPORT

/* Macro for CAN FD Support, undefine this to use as Std CAN */
#ifdef ENABLE_CANFD_SUPPORT
#define APP_MCAN_BUFFER_SIZE                     (MCAN_DATA_SIZE_64BYTES)
#define FILE_PKT_SIZE                            (63U)
#else
/* Payload size is 8 bytes */
#define APP_MCAN_BUFFER_SIZE                     (MCAN_DATA_SIZE_8BYTES)
#define FILE_PKT_SIZE                            (7U)
#endif

/* Extended Id configured in this app */
#define APP_MCAN_EXT_ID_MASK                     (0x1FFFFFFFU)

/* Maximum STD Filter Element can be configured is 128 */
#define APP_MCAN_STD_ID_FILTER_CNT               (0U)
/* Maximum EXT Filter Element can be configured is 64 */
#define APP_MCAN_EXT_ID_FILTER_CNT               (1U)

/* Maximum TX Buffer + TX FIFO, combined can be configured is 32 */
#define APP_MCAN_TX_BUFF_CNT                     (0U)
#define APP_MCAN_TX_FIFO_CNT                     (1U)

/* Maximum TX Event FIFO can be configured is 32 */
#define APP_MCAN_TX_EVENT_FIFO_CNT               (0U)

/* Maximum RX FIFO 0 can be configured is 1 */
#define APP_MCAN_FIFO_0_CNT                      (1U)
#define APP_MCAN_FIFO_1_CNT                      (0U)

/* Extended Id configured in this app */
#define APP_MCAN_EXT_ID                          (0xC0U)
#define APP_MCAN_EXT_ID_MASK                     (0x1FFFFFFFU)

/* Classic Bit Mask Filter */
#define APP_MCAN_CLASSIC_BIT_MASK                (0xFFFFFFFFU)

int32_t Bootloader_CANReceiveFile(uint32_t *fileSize, uint8_t *dstBuf, uint32_t *run);

int32_t Bootloader_CANTransmitResp(uint8_t *src);

void Bootloader_CANInit(uint64_t systemAddr);

#ifdef __cplusplus
}
#endif

#endif /* BOOTLOADER_CAN_H_ */