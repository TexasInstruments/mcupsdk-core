/*
 * Copyright (C) 2022 Texas Instruments Incorporated
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
 *  \file   lin.c
 *
 *  \brief  This file contains the implementation of LIN driver
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* This is needed for memset/memcpy */
#include <string.h>
#include <stdint.h>
#include <drivers/lin.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/csl_types.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/TaskP.h>
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void LIN_initModule(uint32_t base)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));

    /*
       Reset LIN module
       Release from hard reset
    */
    LIN_disableModule(base);
    LIN_enableModule(base);

    /* Enter Software Reset State */
    LIN_enterSoftwareReset(base);

    /* Enable LIN Mode */
    LIN_disableSCIMode(base);

    /* Set LIN mode to Commander */
    LIN_setLINMode(base, LIN_MODE_LIN_COMMANDER);

    /* Enable Fixed baud rate mode */
    LIN_disableAutomaticBaudrate(base);

    /* Use the set frame length and not ID4/ID5 bits for length control */
    LIN_setCommMode(base, LIN_COMM_LIN_USELENGTHVAL);

    /* Setup to continue operating on emulation suspend */
    LIN_setDebugSuspendMode(base, LIN_DEBUG_COMPLETE);

    /* Use Enhanced Checksum */
    LIN_setChecksumType(base, LIN_CHECKSUM_ENHANCED);

    /* Message filtering uses responder task ID byte */
    LIN_setMessageFiltering(base, LIN_MSG_FILTER_IDRESPONDER);

    /* Disable Internal loopback for external communication */
    LIN_disableIntLoopback(base);

    /* Enable multi-buffer mode */
    LIN_enableMultibufferMode(base);

    /* Enable parity check on received ID */
    LIN_enableParity(base);

    /* Enable transfer of data to and from the shift registers  */
    LIN_enableDataTransmitter(base);
    LIN_enableDataReceiver(base);

    /* Enable the triggering of checksum compare on extended frames */
    LIN_triggerChecksumCompare(base);

    /* Set LIN interrupts to disabled */
    LIN_disableInterrupt(base, LIN_INT_ALL);

    /* Set Baud Rate Settings - 100MHz Device */
    LIN_setBaudRatePrescaler(base, 96U, 11U);
    LIN_setMaximumBaudRate(base, 100000000U);

    /* Set response field to 1 byte */
    LIN_setFrameLength(base, 1U);

    /* Configure sync field
       Sync break (13 + 5 = 18 Tbits)
       Sync delimiter (1 + 3 = 4 Tbits)
    */
    LIN_setSyncFields(base, 5U, 3U);

    /* Set Mask ID so TX/RX match will always happen */
    LIN_setTxMask(base, 0xFFU);
    LIN_setRxMask(base, 0xFFU);

    /* Disable IODFT testing and external loopback mode */
    LIN_disableExtLoopback(base);

    /* Finally exit SW reset and enter LIN ready state */
    LIN_exitSoftwareReset(base);
}

void LIN_getData(uint32_t base, uint16_t * const data)
{
    uint16_t i;
    uint16_t length;
    uint16_t *pData = data;

    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));

    /* Get the length from the SCIFORMAT register. */
    length = (uint16_t)((HW_RD_REG32_RAW(base + CSL_LIN_SCIFORMAT) &
                        CSL_LIN_SCIFORMAT_LENGTH_MASK) >> CSL_LIN_SCIFORMAT_LENGTH_SHIFT);

    /* Read each 8-bit piece of data. */
    for(i = 0U; i <= length; i++)
    {
        if(pData != 0U)
        {
            *pData = HW_RD_REG8_RAW(base + CSL_LIN_LINRD0 + ((uint32_t)i ^ 3U));
            pData++;
        }
    }
}

void LIN_sendData(uint32_t base, uint16_t *data)
{
    int16_t i;
    uint16_t length;
    uint16_t *pData;

    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));

    /* Get the length from the SCIFORMAT register. */
    length = (uint16_t)((HW_RD_REG32_RAW(base + CSL_LIN_SCIFORMAT) &
                          CSL_LIN_SCIFORMAT_LENGTH_MASK) >> CSL_LIN_SCIFORMAT_LENGTH_SHIFT);

    pData = data + length;

    /* Shift each 8-bit piece of data into the correct register. */
    for(i = (int32_t)length; i >= 0; i--)
    {
        if(pData != 0U)
        {
            HW_WR_REG8_RAW(base + CSL_LIN_LINTD0 + ((uint16_t)i ^ 3U), (uint8_t)*pData);
            pData--;
        }
    }
}
