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
 *  \file   gpio_v1.c
 *
 *  \brief  Low lever APIs performing hardware register writes and reads for
 *          GPIO IP version 1.
 *
 *   This file contains the hardware register write/read APIs for GPIO.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <drivers/gpio.h>
#include <drivers/hw_include/hw_types.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void GPIO_moduleEnable(uint32_t baseAddr)
{
    /* Clearing the DISABLEMODULE bit in the Control(CTRL) register. */
    CSL_REG32_WR(baseAddr + CSL_GPIO_GIOGCR, CSL_GPIO_GIOGCR_RESET_MAX);
}

int32_t GPIO_setTrigType(uint32_t baseAddr, uint32_t pinNum, uint32_t polLevel)
{
    int32_t  status = SystemP_SUCCESS;
    uint32_t portNum = GPIO_GET_PORT_NUM(pinNum);
    uint32_t pinIndex = GPIO_GET_PIN_INDEX(pinNum);
        
    if (portNum > (GPIO_MAX_INTERRUPT_PORT-1))
    {
        return SystemP_FAILURE;
    }
    uint32_t regValue = CSL_REG32_RD(baseAddr + CSL_GPIO_GIOPOL);
    regValue &= ~(1U << ((portNum * 8U) + pinIndex));
    regValue |= (uint32_t) polLevel << ((portNum * 8U) + pinIndex);
    CSL_REG32_WR(baseAddr + CSL_GPIO_GIOPOL, regValue);
    
    return status;
}

int32_t GPIO_ignoreOrHonorPolarity(uint32_t baseAddr, uint32_t pinNum, uint32_t polLevel)
{
    int32_t  status = SystemP_SUCCESS;
    uint32_t portNum = GPIO_GET_PORT_NUM(pinNum);
    uint32_t pinIndex = GPIO_GET_PIN_INDEX(pinNum);
    
    if (portNum > (GPIO_MAX_INTERRUPT_PORT-1))
    {
        return SystemP_FAILURE;
    }
    
    uint32_t regValue = CSL_REG32_RD(baseAddr + CSL_GPIO_GIOINTDET);
    if((polLevel == GPIO_TRIG_TYPE_FALL_EDGE) || (polLevel == GPIO_TRIG_TYPE_RISE_EDGE))
    {
        regValue &= ~(1U << ((portNum * 8U) + pinIndex));
    }
    else
    {
        regValue |= 1U << ((portNum * 8U) + pinIndex); 
    }
    CSL_REG32_WR(baseAddr + CSL_GPIO_GIOINTDET, regValue);
    
    return status;
}

int32_t GPIO_enableInterrupt(uint32_t baseAddr, uint32_t pinNum)
{
    int32_t  status = SystemP_SUCCESS;
    uint32_t portNum = GPIO_GET_PORT_NUM(pinNum);
    uint32_t pinIndex = GPIO_GET_PIN_INDEX(pinNum);
    
    if (portNum > (GPIO_MAX_INTERRUPT_PORT-1))
    {
        return SystemP_FAILURE;
    }
    uint32_t regValue = (uint32_t) 1 << ((portNum * 8U) + pinIndex);
    CSL_REG32_WR(baseAddr + CSL_GPIO_GIOENASET, regValue);
    
    return status;
}

int32_t GPIO_disableInterrupt(uint32_t baseAddr, uint32_t pinNum)
{
    int32_t  status = SystemP_SUCCESS;
    uint32_t portNum = GPIO_GET_PORT_NUM(pinNum);
    uint32_t pinIndex = GPIO_GET_PIN_INDEX(pinNum);
    
    if (portNum > (GPIO_MAX_INTERRUPT_PORT-1))
    {
        return SystemP_FAILURE;
    }
    uint32_t regValue = (uint32_t) 1 << ((portNum * 8U) + pinIndex);
    CSL_REG32_WR(baseAddr + CSL_GPIO_GIOENACLR, regValue);
    
    return status;
}

int32_t GPIO_clearInterrupt(uint32_t baseAddr, uint32_t pinNum)
{
    int32_t  status = SystemP_SUCCESS;
    uint32_t portNum = GPIO_GET_PORT_NUM(pinNum);
    uint32_t pinIndex = GPIO_GET_PIN_INDEX(pinNum);
    
    if (portNum > (GPIO_MAX_INTERRUPT_PORT-1))
    {
        return SystemP_FAILURE;
    }
    uint32_t regValue = (uint32_t) 1 << ((portNum * 8U) + pinIndex);
    CSL_REG32_WR(baseAddr + CSL_GPIO_GIOFLG, regValue);
    
    return status;
}

int32_t GPIO_markHighLowLevelInterrupt(uint32_t baseAddr, uint32_t pinNum, uint32_t interruptLevel)
{
    int32_t  status = SystemP_SUCCESS;
    uint32_t portNum = GPIO_GET_PORT_NUM(pinNum);
    uint32_t pinIndex = GPIO_GET_PIN_INDEX(pinNum);
    
    if (portNum > (GPIO_MAX_INTERRUPT_PORT-1))
    {
        return SystemP_FAILURE;
    }
    if(interruptLevel == GPIO_INTR_LEVEL_HIGH)
    {
        uint32_t regValue = CSL_REG32_RD(baseAddr + CSL_GPIO_GIOLVLSET);
        regValue |= (uint32_t) 1 << ((portNum * 8U) + pinIndex);
        CSL_REG32_WR(baseAddr + CSL_GPIO_GIOLVLSET, regValue);
    }
    else 
    {
        uint32_t regValue = CSL_REG32_RD(baseAddr + CSL_GPIO_GIOLVLCLR);
        regValue |= (uint32_t) 1 << ((portNum * 8U) + pinIndex);
        CSL_REG32_WR(baseAddr + CSL_GPIO_GIOLVLCLR, regValue);
    }
    return status;
}

uint32_t GPIO_getHighLowLevelPendingInterrupt(uint32_t baseAddr, uint32_t interruptLevel)
{
    uint32_t regValue;
    if(interruptLevel == GPIO_INTR_LEVEL_HIGH)
    {
        regValue   = CSL_REG32_RD(baseAddr + CSL_GPIO_GIOOFFA); 
    }
    else
    {
        regValue = (CSL_REG32_RD(baseAddr + CSL_GPIO_GIOOFFB));
    }
    return regValue;
}

void GPIO_setDirMode(uint32_t baseAddr, uint32_t pinNum, uint32_t pinDir)
{
    uint32_t portNum, pinIndex;
    
    portNum = GPIO_GET_PORT_NUM(pinNum);
    uint32_t regValue = CSL_REG32_RD(baseAddr + CSL_GPIO_GIODIR(portNum));
    
    pinIndex = GPIO_GET_PIN_INDEX(pinNum);
    regValue &= ~(1U << pinIndex);
    regValue |= (uint32_t) pinDir << pinIndex;
    CSL_REG32_WR(baseAddr + CSL_GPIO_GIODIR(portNum), regValue);
}

uint32_t GPIO_pinRead(uint32_t baseAddr, uint32_t pinNum)
{
    uint32_t portNum = GPIO_GET_PORT_NUM(pinNum);
    uint32_t pinIndex = GPIO_GET_PIN_INDEX(pinNum);
    uint32_t regValue = CSL_REG32_RD(baseAddr + CSL_GPIO_GIODIN(portNum));
    regValue &= 1U << pinIndex;
    return (regValue >> pinIndex);
}

void GPIO_pinWriteHigh(uint32_t baseAddr, uint32_t pinNum)
{
    uint32_t portNum = GPIO_GET_PORT_NUM(pinNum);
    uint32_t pinIndex = GPIO_GET_PIN_INDEX(pinNum);
    CSL_REG32_WR(baseAddr + CSL_GPIO_GIOSET(portNum), (1U << pinIndex));
}

void GPIO_pinWriteLow(uint32_t baseAddr, uint32_t pinNum)
{
    uint32_t portNum = GPIO_GET_PORT_NUM(pinNum);
    uint32_t pinIndex = GPIO_GET_PIN_INDEX(pinNum);
    CSL_REG32_WR(baseAddr + CSL_GPIO_GIOCLR(portNum), (1U << pinIndex));
}

void GPIO_setOpenDrainStatus(uint32_t baseAddr, uint32_t pinNum, uint32_t openDrainStatus)
{
    uint32_t portNum = GPIO_GET_PORT_NUM(pinNum);
    uint32_t pinIndex = GPIO_GET_PIN_INDEX(pinNum);
    
    uint32_t regValue = CSL_REG32_RD(baseAddr + CSL_GPIO_GIOPDR(portNum));
    regValue &= ~(1U << pinIndex);
    regValue |= (uint32_t) openDrainStatus << pinIndex;
    CSL_REG32_WR(baseAddr + CSL_GPIO_GIOPDR(portNum), regValue);
}
