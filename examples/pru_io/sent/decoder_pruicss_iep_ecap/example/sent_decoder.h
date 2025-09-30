/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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

#include <stdio.h>
#include <string.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/pruicss.h>
#include <icss_intc_defines.h>

/**
 *  Value to be set in ICSSM_PRU0_GPIO_OUT_CTRL and ICSSM_PRU1_GPIO_OUT_CTRL registers
 *  when using pins in GPIO mode.
 *  1 - Input
 *  0- Output
 *  This configures PRG0_PRUx_GPIO0, PRG0_PRUx_GPIO1, PRG0_PRUx_GPIO2, PRG0_PRUx_GPIO3,
 *  PRG0_PRUx_GPIO4, PRG0_PRUx_GPIO5, PRG0_PRUx_GPIO6, PRG0_PRUx_GPIO8 as input pins.
 **/
#define MSS_CTRL_ICSSM_PRU_GPIO_OUT_CTRL_VALUE  (0x17F)

#define NUM_SENT_CHANNELS           (6U)

#define DATA_READY                          (1<<0)
#define SHORT_SERIAL_MESSAGE_DATA_READY     (1<<1)
#define ENHANCED_SERIAL_MESSAGE_DATA_READY  (1<<2)

/*CRC4 LUT offset*/
#define CRC4_LUT_OFFSET             (0x0000U)

/*CRC6 LUT offset*/
#define CRC6_LUT_OFFSET             (0x0400U)

/*Data ready flag offset located in SRAM*/
#define CH0_DATA_READY_FLAG_OFFSET  (0x00U)
#define CH1_DATA_READY_FLAG_OFFSET  (0x01U)
#define CH2_DATA_READY_FLAG_OFFSET  (0x02U)
#define CH3_DATA_READY_FLAG_OFFSET  (0x03U)
#define CH4_DATA_READY_FLAG_OFFSET  (0x04U)
#define CH5_DATA_READY_FLAG_OFFSET  (0x05U)


/*Offset for Data buffer of channels located in SRAM*/
#define CH0_DATA_OFFSET             (0x40U)
#define CH1_DATA_OFFSET             (0x4CU)
#define CH2_DATA_OFFSET             (0x58U)
#define CH3_DATA_OFFSET             (0x64U)
#define CH4_DATA_OFFSET             (0x70U)
#define CH5_DATA_OFFSET             (0x7CU)

#define CH0_SERIAL_MSG_DATA_BASE    (0x80)
#define CH1_SERIAL_MSG_DATA_BASE    (0x84)
#define CH2_SERIAL_MSG_DATA_BASE    (0x88)
#define CH3_SERIAL_MSG_DATA_BASE    (0x8C)
#define CH4_SERIAL_MSG_DATA_BASE    (0x90)
#define CH5_SERIAL_MSG_DATA_BASE    (0x94)

/*Offset for data nibbles*/
#define CONFIG_TICK_TIME_OFFSET     (0x0U)
#define STATUS_COM_BIT_OFFSET       (0x2U)
#define DATA0_OFFSET                (0x3U)
#define DATA1_OFFSET                (0x4U)
#define DATA2_OFFSET                (0x5U)
#define DATA3_OFFSET                (0x6U)
#define DATA4_OFFSET                (0x7U)
#define DATA5_OFFSET                (0x8U)
#define CRC_OFFSET                  (0x9U)
#define ERROR_STATUS_OFFSET         (0xAU)

/*Offsets, masks and shift values for short serial message data nibbles*/
#define SHORT_SERIAL_MESSAGE_BYTE0                (0x0U)
#define SHORT_SERIAL_MESSAGE_BYTE1                (0x1U)
#define SHORT_SERIAL_MESSAGE_BYTE0_DATA1_MASK     (0xF0)
#define SHORT_SERIAL_MESSAGE_BYTE0_DATA1_SHIFT    (0x4)
#define SHORT_SERIAL_MESSAGE_BYTE0_CRC_MASK       (0x0F)
#define SHORT_SERIAL_MESSAGE_BYTE0_CRC_SHIFT      (0x0)
#define SHORT_SERIAL_MESSAGE_BYTE1_ID_MASK        (0xF0)
#define SHORT_SERIAL_MESSAGE_BYTE1_ID_SHIFT       (0x4)
#define SHORT_SERIAL_MESSAGE_BYTE1_DATA0_MASK     (0x0F)
#define SHORT_SERIAL_MESSAGE_BYTE1_DATA0_SHIFT    (0x0)

/*Offsets, masks and shift values for enhanced serial message data nibbles*/
/* Word = 4 bytes for following macros */
#define ENHANCED_SERIAL_MESSAGE_WORD0                   (0x0U)
#define ENHANCED_SERIAL_MESSAGE_WORD1                   (0x4U)

#define ENHANCED_SERIAL_MESSAGE_WORD0_CRC_MASK          (0x0003F000)
#define ENHANCED_SERIAL_MESSAGE_WORD0_CRC_SHIFT         (0xC)

#define ENHANCED_SERIAL_MESSAGE_WORD0_DATA_MASK         (0x00000FFF)
#define ENHANCED_SERIAL_MESSAGE_WORD0_DATA_SHIFT        (0x0)

#define ENHANCED_SERIAL_MESSAGE_WORD1_CONFIG_MASK       (0x00000400)
#define ENHANCED_SERIAL_MESSAGE_WORD1_CONFIG_SHIFT      (0xA)

#define ENHANCED_SERIAL_MESSAGE_WORD1_C0_ID_LOW_MASK    (0x0000001E)
#define ENHANCED_SERIAL_MESSAGE_WORD1_C0_ID_LOW_SHIFT   (0x1)

#define ENHANCED_SERIAL_MESSAGE_WORD1_C0_ID_HIGH_MASK   (0x000003C0)
#define ENHANCED_SERIAL_MESSAGE_WORD1_C0_ID_HIGH_SHIFT  (0x6)

#define ENHANCED_SERIAL_MESSAGE_WORD1_C1_DATA_MASK      (0x0000001E)
#define ENHANCED_SERIAL_MESSAGE_WORD1_C1_DATA_SHIFT     (0x1)

#define ENHANCED_SERIAL_MESSAGE_WORD1_C1_ID_MASK        (0x000003C0)
#define ENHANCED_SERIAL_MESSAGE_WORD1_C1_ID_SHIFT       (0x6)

/** \brief Global Structure pointer holding PRUSS1 memory Map. */
PRUICSS_Handle gPruIcss0Handle;

/**
 * @brief SENT Wave Struct
 * Sync Pulse: 56 ticks
 * 4 bit Status and Message Pulse: 17-32 tick
 * 4 bit (9:12) Data0 Field: 17-32 ticks
 * 4 bit (5:8) Data1 Field: 17-32 ticks
 * 4 bit (1:4) Data2 Field: 17-32 ticks
 * 4 bit (9-12) Data3 Field: 17-32 ticks
 * 4 bit (5-8) Data4 Field: 17-32 ticks
 * 4 bit (1-4) Data5 Field: 17-32 ticks
 * 4 bit CRC: 17-32 ticks
 *
 */
typedef struct Sent_Obj_s
{
    /*Tick Period Config*/
    uint16_t ConfigTickTime;
    /*Status & Comm Data 0*/
    uint8_t  StatusComBit;
    /*Data 0*/
    uint8_t Data0;
    /*Data 1*/
    uint8_t Data1;
    /*Data 2*/
    uint8_t Data2;
    /*Data 3*/
    uint8_t Data3;
    /*Data 4*/
    uint8_t Data4;
    /*Data 5*/
    uint8_t Data5;
    /*CRC*/
    uint8_t CRC;
    /*Channel Status Code*/
    uint16_t error_status;
} Sent_Obj;

typedef struct Sent_ShortSerialMessage_s
{
    /*Message ID */
    uint8_t MessageId;
    /*Data 0*/
    uint8_t Data0;
    /*Data 1*/
    uint8_t Data1;
    /*CRC*/
    uint8_t CRC;
} Sent_ShortSerialMessage;

typedef struct Sent_EnhancedSerialMessage_s
{
    /*Configuration Bit */
    uint8_t ConfigBit;
    /*Message ID*/
    uint8_t MessageId;
    /*Data*/
    uint16_t Data;
    /*CRC*/
    uint8_t CRC;
    /*Reserved*/
    uint8_t reserved0;
    uint8_t reserved1;
    uint8_t reserved2;
} Sent_EnhancedSerialMessage;

/*!
 * \brief SENT FRame handle.
 *
 * SENT Frame opaque handle used to call any SENT related APIs.
 */
typedef struct Sent_Obj_s *Sent_Handle;
