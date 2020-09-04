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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <drivers/gpio.h>
#include <drivers/mibspi.h>
#include <drivers/edma.h>
#include <kernel/dpl/AddrTranslateP.h>
#include "ti_drivers_open_close.h"

/* size used for the test*/
#define APP_MSGSIZE                 4   

static uint8_t BoardDiag_mcanReadPmicReg(MIBSPI_Handle handle, uint8_t regOffset)
{
    uint8_t txBuffer[APP_MSGSIZE];
    uint8_t rxBuffer[APP_MSGSIZE];
    uint8_t regValue;
    MIBSPI_Transaction spiTransaction;

    /* Configure Data Transfer */
    spiTransaction.count = APP_MSGSIZE-1;
    spiTransaction.txBuf = txBuffer;
    spiTransaction.rxBuf = rxBuffer;
    spiTransaction.slaveIndex = 0;
    txBuffer[0] = regOffset;
    // Indicate PMIC a read sequence */
    txBuffer[1] = 0x10;
    txBuffer[2] = 0;
    /* Start Data Transfer */
    MIBSPI_transfer(handle, &spiTransaction);
    
    /*PMIC register value */
    regValue = rxBuffer[2];

    return regValue;
}

static void BoardDiag_mcanWritePmicReg(MIBSPI_Handle handle, uint8_t regAddr, uint8_t val)
{
    uint8_t txBuffer[APP_MSGSIZE];
    MIBSPI_Transaction spiTransaction;

    /* Configure Data Transfer */
    spiTransaction.count = APP_MSGSIZE-1;
    spiTransaction.txBuf = txBuffer;
    spiTransaction.rxBuf = NULL;
    spiTransaction.slaveIndex = 0;
    txBuffer[0] = regAddr;
    /* Indicate PMIC a write sequence */
    txBuffer[1] = 0;
    /* Write data */
    txBuffer[2] = val;

    CacheP_wbInv((void *) txBuffer, 3,CacheP_TYPE_ALLD);
    /* Start Data Transfer */
    MIBSPI_transfer(handle, &spiTransaction);
}

int32_t BoardDiag_mcanConfigSTB(MIBSPI_Handle handle)
{
    uint8_t regValue;
    uint8_t regAddr;
    int32_t status = SystemP_SUCCESS;

    /* Read the PMIC register value */
    regValue = BoardDiag_mcanReadPmicReg(handle, 0xA);

    regAddr = 0xA;
    regValue = 0x9B;
    ClockP_sleep(4);
    BoardDiag_mcanWritePmicReg(handle, regAddr, regValue);
    
    regAddr = 0x62;
    regValue = 0;
    ClockP_sleep(4);
    BoardDiag_mcanWritePmicReg(handle, regAddr, regValue);
    
    regAddr = 0x62;
    regValue = BoardDiag_mcanReadPmicReg(handle, regAddr);
    
    regAddr = 0x1B;
    regValue = BoardDiag_mcanReadPmicReg(handle, regAddr);
    
    regAddr  = 0x1B;
    regValue = regValue | 0x2;
    ClockP_sleep(4);
    BoardDiag_mcanWritePmicReg(handle, regAddr, regValue);
    
    regAddr = 0x1B;
    regValue = BoardDiag_mcanReadPmicReg(handle, regAddr);
    
    regAddr = 0x1B;
    regValue = BoardDiag_mcanReadPmicReg(handle, regAddr);
    if(regValue  == 0)
    {
        status = SystemP_FAILURE;
    }

    return status;
}

void test_mcanEnableTransceiver(void)
{
    int32_t retVal;

    /* configure CAN STB pin through PMIC */
    retVal = BoardDiag_mcanConfigSTB(gMibspiHandle[CONFIG_MIBSPI0]);
    if(retVal != 0)
    {
        DebugP_logError("CAN STB Pin Configurations Failed!!\n");
        DebugP_assert(FALSE);
    }
}
