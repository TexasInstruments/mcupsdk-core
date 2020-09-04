/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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

#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <drivers/crc.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
 * This example demonstrates Cyclic Redundancy Check(CRC) in full CPU mode for
 * Channel number 1. CRC signature is calculated on a frame that is stored in
 * memory and compared against pre-calculated CRC signature value.
 */

/* CRC configuration parameters */
#define APP_CRC_CHANNEL                     (CRC_CHANNEL_1)

/* Pre-calculated crc signature value for given data pattern */
#define APP_CRC_REFERENCE_SIGN_VAL_L        (0xAEF33083U)
#define APP_CRC_REFERENCE_SIGN_VAL_H        (0x3668F7EAU)

/* Frame details - used as reference data and
   it must be multiple of APP_CRC_PATTERN_SIZE */
#define APP_USER_DATA_SIZE                  ((uint32_t) 40000U)

/* CRC pattern size updated to 32 bits from 64 bits as CPU can copy only 32 bits at a time */
#define APP_CRC_PATTERN_SIZE                ((uint32_t) 4U)
#define APP_CRC_PATTERN_CNT                 (APP_USER_DATA_SIZE/APP_CRC_PATTERN_SIZE)
#define APP_CRC_SECT_CNT                    ((uint32_t) 1U)
#define APP_CRC_WATCHDOG_PRELOAD_VAL        ((uint32_t) 0U)
#define APP_CRC_BLOCK_PRELOAD_VAL           ((uint32_t) 0U)

static uint32_t gSrcBuffer[APP_CRC_PATTERN_CNT];

void crc_full_cpu_main(void *args)
{
    uint32_t                loopCnt, patternCnt, baseAddr;
    CRC_Channel_t           chNumber;
    CRC_Signature           signVal;
    CRC_SignatureRegAddr    psaSignRegAddr;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("CRC Full CPU Test Started ...\r\n");

    /* Configure CRC parameters */
    baseAddr           = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
    patternCnt         = APP_CRC_PATTERN_CNT;
    chNumber           = APP_CRC_CHANNEL;

    /* Initialize source memory with reference data */
    for(loopCnt = 0; loopCnt < patternCnt; loopCnt++)
    {
        gSrcBuffer[loopCnt] = loopCnt;
    }

    /* Get CRC PSA signature register address */
    CRC_getPSASigRegAddr(baseAddr, chNumber, &psaSignRegAddr);

    /* Initialize and Configure CRC channel */
    CRC_initialize(baseAddr, chNumber, APP_CRC_WATCHDOG_PRELOAD_VAL, APP_CRC_BLOCK_PRELOAD_VAL);

    CRC_configure(baseAddr, chNumber, patternCnt, APP_CRC_SECT_CNT, CRC_OPERATION_MODE_FULLCPU);

    /* Reset the CRC channel*/
    CRC_channelReset(baseAddr, chNumber);

    /* compute the CRC by writing the data buffer on which CRC computation is needed */
    for (loopCnt = 0; loopCnt < patternCnt ; loopCnt++)
    {
        HW_WR_REG32(psaSignRegAddr.regL, gSrcBuffer[loopCnt]);
    }

    /* Fetch CRC signature value */
    CRC_getPSASig(baseAddr, chNumber, &signVal);

    /* Compare CRC signature value against reference CRC signature */
    if(!((signVal.regH == APP_CRC_REFERENCE_SIGN_VAL_H) &&
         (signVal.regL == APP_CRC_REFERENCE_SIGN_VAL_L)))
    {
        /* Signature value not matches */
        DebugP_log("CRC signature value does not match with pre-calculated value!!\r\n");
        DebugP_log("Some tests have failed!!\r\n");
    }
    else
    {
        DebugP_log("Full CPU mode CRC Test Passed!!\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }

    Board_driversClose();
    Drivers_close();
}
