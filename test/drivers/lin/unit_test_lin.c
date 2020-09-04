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

#include <drivers/lin.h>
#include "ti_drivers_config.h"


/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define LIN_RAM_BASE  0x70071000

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/* LIN FUNCTIONS */
void Unit_setupTxMask(uint16_t mask)
{
    LIN_setTxMask(CONFIG_LIN1_BASE_ADDR, mask);
}

void Unit_setupRxMask(uint16_t mask)
{
    LIN_setRxMask(CONFIG_LIN1_BASE_ADDR, mask);
}

void Unit_setupRxID(uint32_t ID)
{
    HW_WR_FIELD32_RAW((LIN_RAM_BASE + CSL_LIN_LINID), CSL_LIN_LINID_RECEIVEDID_MASK, CSL_LIN_LINID_RECEIVEDID_SHIFT, ID);
}

void Unit_setupClearIntStatus()
{
    HW_WR_REG32_RAW((LIN_RAM_BASE + CSL_LIN_SCIFLR), 0U);
}

void Unit_setupInterruptLevel(uint32_t ints)
{
    LIN_setInterruptLevel1(CONFIG_LIN1_BASE_ADDR, ints);
}

void Unit_setupInterrupts(uint32_t flags)
{
    LIN_enableInterrupt(CONFIG_LIN1_BASE_ADDR, flags);
}

void Unit_setupModuleErrors(uint32_t errors)
{
    LIN_disableModuleErrors(CONFIG_LIN1_BASE_ADDR, LIN_ALL_ERRORS);
    LIN_enableModuleErrors(CONFIG_LIN1_BASE_ADDR, errors);
}

/////SCI-LIN Functions
void Unit_initSCIModule()
{
    /* Reset LIN module
       Release from hard reset
    */
    LIN_disableModule(CONFIG_LIN1_BASE_ADDR);
    LIN_enableModule(CONFIG_LIN1_BASE_ADDR);

    /* Enter Software Reset State */
    LIN_enterSoftwareReset(CONFIG_LIN1_BASE_ADDR);

    /* Enable SCI Mode */
    LIN_enableSCIMode(CONFIG_LIN1_BASE_ADDR);

    LIN_setSCICommMode(CONFIG_LIN1_BASE_ADDR, LIN_COMM_SCI_IDLELINE);
    LIN_setSCIStopBits(CONFIG_LIN1_BASE_ADDR, LIN_SCI_STOP_ONE);
    LIN_setSCICharLength(CONFIG_LIN1_BASE_ADDR, 8U);

    /* Setup to continue operating on emulation suspend */
    LIN_setDebugSuspendMode(CONFIG_LIN1_BASE_ADDR, LIN_DEBUG_COMPLETE);

    /* Disable Internal loopback for external communication */
    LIN_enableIntLoopback(CONFIG_LIN1_BASE_ADDR);

    /* Enable multi-buffer mode */
    LIN_enableMultibufferMode(CONFIG_LIN1_BASE_ADDR);

    /* Enable parity check on received ID */
    LIN_enableSCIParity(CONFIG_LIN1_BASE_ADDR, LIN_SCI_PAR_EVEN);

    /* Enable transfer of data to and from the shift registers */
    LIN_enableDataTransmitter(CONFIG_LIN1_BASE_ADDR);
    LIN_enableDataReceiver(CONFIG_LIN1_BASE_ADDR);

    /* Set LIN interrupts to disabled */
    LIN_disableSCIInterrupt(CONFIG_LIN1_BASE_ADDR, LIN_SCI_INT_ALL);

    /* Set Baud Rate Settings - 100MHz Device */
    LIN_setBaudRatePrescaler(CONFIG_LIN1_BASE_ADDR, 96U, 11U);

    /* Set response field to 1 byte */
    LIN_setSCIFrameLength(CONFIG_LIN1_BASE_ADDR, 8U);

    /* Disable IODFT testing and external loopback mode */
    LIN_disableExtLoopback(CONFIG_LIN1_BASE_ADDR);

    /* Finally exit SW reset and enter LIN ready state */
    LIN_exitSoftwareReset(CONFIG_LIN1_BASE_ADDR);
}

void Unit_enableSCIParity(void)
{
    LIN_enableSCIParity(CONFIG_LIN1_BASE_ADDR, LIN_SCI_PAR_EVEN);
}

void Unit_sendSCIDataNonBlock(uint16_t data)
{
    HW_WR_REG32_RAW(LIN_RAM_BASE + CSL_LIN_SCIED, data);
    HW_WR_REG32_RAW(LIN_RAM_BASE + CSL_LIN_SCIRD, data);
}

void Unit_sendSCIDataBlock(uint16_t data)
{
    HW_WR_REG32_RAW(LIN_RAM_BASE + CSL_LIN_SCIFLR, HW_RD_REG32_RAW(LIN_RAM_BASE + CSL_LIN_SCIFLR)|CSL_LIN_SCIFLR_RXRDY_MASK);

    HW_WR_REG32_RAW(LIN_RAM_BASE + CSL_LIN_SCIED, data);
    HW_WR_REG32_RAW(LIN_RAM_BASE + CSL_LIN_SCIRD, data);
}

void Unit_enableSCIErrors(uint16_t errors)
{
    LIN_disableSCIModuleErrors(CONFIG_LIN1_BASE_ADDR, LIN_SCI_ALL_ERRORS);
    LIN_enableSCIModuleErrors(CONFIG_LIN1_BASE_ADDR, errors);
}

void Unit_enableExtLoopback(void)
{
    LIN_enableExtLoopback(CONFIG_LIN1_BASE_ADDR, LIN_LOOPBACK_DIGITAL, LIN_ANALOG_LOOP_NONE);
}

void Unit_setupSCIInterrupts(uint32_t flags)
{
    LIN_enableSCIInterrupt(CONFIG_LIN1_BASE_ADDR, flags);
}

void Unit_setupSCIInterruptLevel(uint32_t ints)
{
    LIN_setSCIInterruptLevel1(CONFIG_LIN1_BASE_ADDR, ints);
}

void Unit_setupInterruptLine0Offset(uint16_t value)
{
    HW_WR_REG32_RAW(LIN_RAM_BASE + CSL_LIN_SCIINTVECT0, value);
}

void Unit_setupInterruptLine1Offset(uint16_t value)
{
    HW_WR_REG32_RAW(LIN_RAM_BASE + CSL_LIN_SCIINTVECT1, value);
}
