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
#include <string.h>
#include <stdio.h>
#include <drivers/gpio.h>
#include <drivers/mibspi.h>
#include <drivers/edma.h>
#include <kernel/dpl/AddrTranslateP.h>
#include "ti_drivers_open_close.h"
#include <drivers/pinmux.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* PMIC configuration message size used using MIBSPIB interface */
#define APP_MSGSIZE                 (3U)   

/* PMIC Register LOCK offset */
#define PMIC_REG_LOCK_OFFSET        (0xAU)
/* PMIC Register Lock value: Other then 9B value */
#define PMIC_REG_LOCK_VALUE         (0x0U)
/* PMIC Register unlock value */
#define PMIC_REG_UNLOCK_VALUE       (0x9BU)
/* PMIC Register CRC configuration offset */
#define PMIC_REG_CRC_OFFSET         (0x62U)
/* PMIC Register CRC configuration value */
#define PMIC_REG_CRC_VALUE          (0x0U)
/* PMIC Register Interface configuration offset */
#define PMIC_REG_INTF_OFFSET        (0x1BU)
/* PMIC Interface configuration NINT polarity value */
#define PMIC_REG_INTF_NINT_VALUE    (0x2U)

static uint8_t BoardDiag_mcanReadPmicReg(MIBSPI_Handle handle, uint8_t regOffset)
{
    uint8_t tx[APP_MSGSIZE];
    uint8_t rx[APP_MSGSIZE];
    MIBSPI_Transaction transaction;

    memset(&transaction, 0, sizeof(transaction));

    /* Configure Data Transfer */
    transaction.count = APP_MSGSIZE;
    transaction.txBuf = tx;
    transaction.rxBuf = rx;
    transaction.slaveIndex = 0;
    /* Single read transmissions consists of 24bit:
     * Bits 0-7  : Register Address
     * Bits 8-10 : Page address for the register
     * Bit  11   : For Read, value should be 1     
     * Bits 12-15: Reserved 
     * Bits 16-23: Value Read from the PMIC 
     */     
    tx[0] = regOffset;
    tx[1] = 0x10;
    tx[2] = 0;
    
    CacheP_wbInv((void *) tx, APP_MSGSIZE, CacheP_TYPE_ALL);
  
    /* Start Data Transfer */
    MIBSPI_transfer(handle, &transaction);
    /* Invalidate the receive buffer */
    CacheP_inv((void *) rx, APP_MSGSIZE, CacheP_TYPE_ALL);

    return rx[2];
 
}

static void BoardDiag_mcanWritePmicReg(MIBSPI_Handle handle, uint8_t regAddr, uint8_t val)
{
 
    uint8_t tx[APP_MSGSIZE];
    MIBSPI_Transaction transaction;

    memset(&transaction, 0, sizeof(transaction));

    /* Configure Data Transfer */
    transaction.count = APP_MSGSIZE;
    transaction.txBuf = tx;
    transaction.rxBuf = NULL;
    transaction.slaveIndex = 0;
    /* Single write transmissions consists of 24bit:
     * Bits 0-7  : Register Address
     * Bits 8-10 : Page address for the register
     * Bit  11   : For Write, value should be 0     
     * Bits 12-15: Reserved 
     * Bits 16-23: Value to be written to the register 
     */
    tx[0] = regAddr;
    tx[1] = 0;
    tx[2] = val;

    CacheP_wbInv((void *) tx, APP_MSGSIZE, CacheP_TYPE_ALL);
    /* Start Data Transfer */
    MIBSPI_transfer(handle, &transaction);
}

int32_t BoardDiag_mcanConfigSTB(MIBSPI_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    uint8_t regValue;
    uint8_t regAddr;

    /* Unlock the registers */
    regAddr = PMIC_REG_LOCK_OFFSET;
    regValue = PMIC_REG_UNLOCK_VALUE;
    BoardDiag_mcanWritePmicReg(handle, regAddr, regValue);
    ClockP_usleep(10000);
    
    /* Read Register Lock Status  
     * 1: Registers are locked and configuration registers are read only
     * 0: Registers are unlocked and configuration registers can be written
     */
    regAddr = PMIC_REG_LOCK_OFFSET;
    regValue = BoardDiag_mcanReadPmicReg(handle, regAddr);
    
    /* Check for unlock register successful */
    if(regValue != 0)
    {
        DebugP_logError("Register unlock failed and check the mibspi bitrate!!\n");
        DebugP_assert(FALSE);
    }
    
    /* Configure the CRC */ 
    regAddr = PMIC_REG_CRC_OFFSET;
    regValue = PMIC_REG_CRC_VALUE;
    BoardDiag_mcanWritePmicReg(handle, regAddr, regValue);
    ClockP_usleep(10000);
    
    /* Read Interface Config register */
    regAddr = PMIC_REG_INTF_OFFSET;
    regValue = BoardDiag_mcanReadPmicReg(handle, regAddr);
   
    /* Configure nINT active high */ 
    regAddr  = PMIC_REG_INTF_OFFSET;
    regValue = regValue | PMIC_REG_INTF_NINT_VALUE;
    BoardDiag_mcanWritePmicReg(handle, regAddr, regValue);
    ClockP_usleep(10000);
    
    /* Read back nINT bit value and validate */
    regAddr = PMIC_REG_INTF_OFFSET;
    regValue = BoardDiag_mcanReadPmicReg(handle, regAddr);

    if((regValue & PMIC_REG_INTF_NINT_VALUE) != PMIC_REG_INTF_NINT_VALUE)
    {
       status = SystemP_FAILURE; ;
    }
    
     /* Lock the registers */
    regAddr = PMIC_REG_LOCK_OFFSET;
    regValue = PMIC_REG_LOCK_VALUE;
    BoardDiag_mcanWritePmicReg(handle, regAddr, regValue);
    ClockP_usleep(10000);
    
    /* Read the Lock Status register */
    regAddr = PMIC_REG_LOCK_OFFSET;
    regValue = BoardDiag_mcanReadPmicReg(handle, regAddr);
    
    /* Check for lock register successful */
    if(regValue != 1)
    {
        DebugP_logError("Register lock failed !!\n");
        DebugP_assert(FALSE);
    }
    
    return status;
}

void mcan_enableTransceiver(void)
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
