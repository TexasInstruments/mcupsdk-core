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

/*CRC4 LUT offset*/
#define CRC4_LUT_OFFSET             (0x00U)

/*Data ready flag offset located in SRAM*/
#define CH0_DATA_READY_FLAG_OFFSET  (0x00U)
#define CH1_DATA_READY_FLAG_OFFSET  (0x01U)
#define CH2_DATA_READY_FLAG_OFFSET  (0x02U)
#define CH3_DATA_READY_FLAG_OFFSET  (0x03U)
#define CH4_DATA_READY_FLAG_OFFSET  (0x04U)
#define CH5_DATA_READY_FLAG_OFFSET  (0x05U)
#define CH6_DATA_READY_FLAG_OFFSET  (0x06U)
#define CH7_DATA_READY_FLAG_OFFSET  (0x07U)

/*Offset for Data buffer of channels located in SRAM*/
#define CH0_DATA_OFFSET             (0x40U)
#define CH1_DATA_OFFSET             (0x4CU)
#define CH2_DATA_OFFSET             (0x58U)
#define CH3_DATA_OFFSET             (0x64U)
#define CH4_DATA_OFFSET             (0x70U)
#define CH5_DATA_OFFSET             (0x7CU)
#define CH6_DATA_OFFSET             (0x88U)
#define CH7_DATA_OFFSET             (0x94U)

/*Flag Mask for individual channels*/
#define CH0_FLAG_MASK               (0xFFU)
#define CH1_FLAG_MASK               (0xFF00U)
#define CH2_FLAG_MASK               (0xFF0000U)
#define CH3_FLAG_MASK               (0xFF000000U)
#define CH4_FLAG_MASK               (0xFFU)
#define CH5_FLAG_MASK               (0xFF00U)
#define CH6_FLAG_MASK               (0xFF0000U)
#define CH7_FLAG_MASK               (0xFF000000U)

/*Mask for data nibbles*/
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

/*!
 * \brief SENT FRame handle.
 *
 * SENT Frame opaque handle used to call any SENT related APIs.
 */
typedef struct Sent_Obj_s *Sent_Handle;
