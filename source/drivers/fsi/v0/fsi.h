/*
 * Copyright (C) 2021 Texas Instruments Incorporated
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

/**
 *  \defgroup DRV_FSI_MODULE APIs for FSI
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the FSI module.
 *
 *  @{
 */

/**
 *  \file   v0/fsi.h
 *
 *  \brief  Header file containing various enumerations, structure definitions and function
 *  declarations common for both the FSI TX and FSI RX IP.
 */

#ifndef FSI_V0_H_
#define FSI_V0_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/hw_include/cslr.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* Common defines for both FSI TX and RX */

/**
 * @brief Maximum data length(16 words) for user/software defined data frame
 */
#define FSI_MAX_LEN_NWORDS_DATA     ((uint16_t)0xFU)

/**
 * @brief Maximum value for user data field (8 bits)
 */
#define FSI_MAX_VALUE_USERDATA      ((uint16_t)0xFFU)

/**
 * @brief Maximum value of Buffer pointer offset (4 bits)
 */
#define FSI_MAX_VALUE_BUF_PTR_OFF   ((uint16_t)0xFU)

/**
 * @brief Key value for writing some FSI TX/RX registers
 */
#define FSI_CTRL_REG_KEY            ((uint16_t)0xA5U)


/**
 *  \anchor FSI_DataWidth
 *  \name FSI data width
 *  @{
 */
/**
 * \brief  Data lines used for transmit/receive operation.
 */
typedef uint32_t FSI_DataWidth;
#define FSI_DATA_WIDTH_1_LANE       ((uint32_t)0x0U)
/**< 1 lane data width */
#define FSI_DATA_WIDTH_2_LANE       ((uint32_t)0x1U)
/**< 2 lane data width */
/** @} */

#define FSI_PINGTIMEOUT_ON_HWSWINIT_PING_FRAME ((uint32_t)0x1U)
/**< Ping timeout can reset and restart both on hardware or software
     initiated frame being sent out also based on which mode is selected */
/** @} */

/**
 *  \anchor FSI_ECCComputeWidth
 *  \name ECC Computation width
 *  @{
 */
/**
 * \brief  ECC Computation width - 16 bit or 32 bit
 */
typedef uint32_t FSI_ECCComputeWidth;

#define FSI_32BIT_ECC_COMPUTE       ((uint32_t)0x0U)
/**< ECC Computation width of 32 bit */
#define FSI_16BIT_ECC_COMPUTE       ((uint32_t)0x1U)
/**< ECC Computation width of 16 bit */
/** @} */

/**
 *  \anchor FSI_InterruptNum
 *  \name FSI interrupt line number
 *  @{
 */
/**
 * \brief  Interrupt lines supported in FSI
 *
 * @n Any event on FSI TX or RX can be enabled to trigger interrupt
 *    on 2 interrupt lines to CPU/CLA - INT1 and INT2
 */
typedef uint32_t FSI_InterruptNum;

#define FSI_INT1                    ((uint32_t)0x0U)
/**< Interrupt line 0 */
#define FSI_INT2                    ((uint32_t)0x1U)
/**< Interrupt line 1 */
/** @} */

/**
 *  \anchor FSI_FrameType
 *  \name FSI frame type
 *  @{
 */
/**
 * \brief  FSI frame type
 *
 * Three frame types exist-
 *   Ping: Used for checking line integrity, can be sent by
 *         software or automatically by hardware.
 *   Error: Used typically during error conditions or when one
 *          side wants to signal the other side for attention.
 *   Data: Two subtypes exist based on data-length-
 *         a) \b Fixed (1/2/4/6 words)
 *         b) \b Nwords Software programs number of data words
 *
 * @n 4 bit code for frame types - 0x1, 0x2 and 0x8 to 0xE are reserved
 */
typedef uint32_t FSI_FrameType;

#define FSI_FRAME_TYPE_PING         ((uint32_t)0x0U)
/**< Ping frame type */
#define FSI_FRAME_TYPE_ERROR        ((uint32_t)0xFU)
/**< Error frame type */
#define FSI_FRAME_TYPE_1WORD_DATA   ((uint32_t)0x4U)
/**< 1 word data frame type */
#define FSI_FRAME_TYPE_2WORD_DATA   ((uint32_t)0x5U)
/**< 2 word data frame type */
#define FSI_FRAME_TYPE_4WORD_DATA   ((uint32_t)0x6U)
/**< 4 word data frame type */
#define FSI_FRAME_TYPE_6WORD_DATA   ((uint32_t)0x7U)
/**< 6 word data frame type */
#define FSI_FRAME_TYPE_NWORD_DATA   ((uint32_t)0x3U)
/**< N word data frame type */
/** @} */


/**
 *  \anchor FSI_PingTimeoutMode
 *  \name Ping timeout mode
 *  @{
 */
/**
 * \brief  FSI ping timeout mode
 */
typedef uint32_t FSI_PingTimeoutMode;

/**
 * \brief Ping timeout can reset and restart only on hardware initiated PING
 * frames (PING Watchdog timeout)
 */
#define FSI_PINGTIMEOUT_ON_HWINIT_PING_FRAME   ((uint32_t)0x0U)

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef FSI_V0_H_ */

/** @} */
