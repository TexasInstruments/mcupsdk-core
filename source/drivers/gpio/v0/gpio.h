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
 *  \defgroup DRV_GPIO_MODULE APIs for GPIO
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the GPIO module.
 *
 *  @{
 */

/**
 *  \file v0/gpio.h
 *
 *  \brief GPIO Driver API/interface file.
 *
 *  The GPIO pins are grouped into banks (16 pins per bank and 9 banks per
 *  GPIO instance). This means means that each GPIO module provides up to 144
 *  dedicated general-purpose pins with input and output capabilities
 *
 *  Note: Not all GPIO pins and banks are present in a particular device.
 *  Refer device TRM for actual GPIO instances and pins supported
 */

#ifndef GPIO_V0_H_
#define GPIO_V0_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/cslr_gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 *  \anchor GPIO_PinValue_t
 *  \name GPIO Pin Value
 *  @{
 */
/** \brief GPIO pin is at logic low */
#define GPIO_PIN_LOW                    (0U)
/** \brief GPIO pin is at logic high */
#define GPIO_PIN_HIGH                   (1U)
/** @} */

/**
 *  \anchor GPIO_Direction_t
 *  \name GPIO Direction
 *  @{
 */
#define GPIO_DIRECTION_OUTPUT           (0U)
#define GPIO_DIRECTION_INPUT            (1U)
/** @} */

/**
 *  \anchor GPIO_TrigType_t
 *  \name GPIO Trigger Type
 *  @{
 */
/** \brief No interrupt request on either rising or falling edges on the pin */
#define GPIO_TRIG_TYPE_NONE             (0U)
/** \brief Interrupt request on occurrence of a rising edge on the input pin */
#define GPIO_TRIG_TYPE_RISE_EDGE        (1U)
/** \brief Interrupt request on occurrence of a falling edge on the input pin */
#define GPIO_TRIG_TYPE_FALL_EDGE        (2U)
/** \brief Interrupt request on occurrence of a rising/falling edge on the input pin */
#define GPIO_TRIG_TYPE_BOTH_EDGE        (3U)
/** @} */

/** \brief Maximum number of banks per instance/module */
#define GPIO_MAX_BANKS                  (9U)
/** \brief Maximum number of pins per bank */
#define GPIO_MAX_PIN_PER_BANK           (16U)
/** \brief Maximum number of pins per instance/module */
#define GPIO_MAX_PIN_PER_INSTANCE       (GPIO_MAX_BANKS * GPIO_MAX_PIN_PER_BANK)

/** \brief Number of banks per register */
#define GPIO_BANKS_PER_REG              (2U)
/** \brief Number of pins per register - 32 pins */
#define GPIO_PINS_PER_REG               (GPIO_BANKS_PER_REG * GPIO_MAX_PIN_PER_BANK)

/** \brief Number of pins per register - shift value - used instead of divide operator */
#define GPIO_PINS_PER_REG_SHIFT         (5U)
/** \brief Number of pins per bank - shift value - used instead of divide operator */
#define GPIO_PINS_PER_BANK_SHIFT        (4U)

/** \brief Returns the bank index based on pin number */
#define GPIO_GET_BANK_INDEX(pinNum)     (((uint32_t) pinNum) >> GPIO_PINS_PER_BANK_SHIFT)
/** \brief Returns the register index based on pin number */
#define GPIO_GET_REG_INDEX(pinNum)      (((uint32_t) pinNum) >> GPIO_PINS_PER_REG_SHIFT)
/** \brief Returns the bit position within a register based on pin number */
#define GPIO_GET_BIT_POS(pinNum)        (pinNum - ((GPIO_GET_REG_INDEX(pinNum)) << GPIO_PINS_PER_REG_SHIFT))
/** \brief Returns the bit mask within a register based on pin number */
#define GPIO_GET_BIT_MASK(pinNum)       (((uint32_t) 1U) << GPIO_GET_BIT_POS(pinNum))
/** \brief Returns the bit position within a bank based on pin number */
#define GPIO_GET_BANK_BIT_POS(pinNum)   (pinNum - ((GPIO_GET_BANK_INDEX(pinNum)) << GPIO_PINS_PER_BANK_SHIFT))
/** \brief Returns the bit mask within a bank based on pin number */
#define GPIO_GET_BANK_BIT_MASK(pinNum)  (((uint32_t) 1U) << GPIO_GET_BANK_BIT_POS(pinNum))

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief  This API configures the direction of a specified GPIO pin as being
 *          either input or output
 *
 *  \param  baseAddr    The memory address of the GPIO instance being used
 *  \param  pinNum      The number of the pin in the GPIO instance
 *          'pinNum' can take one of the following values:
 *          (0 <= pinNum <= #GPIO_MAX_PIN_PER_INSTANCE - 1)
 *  \param  pinDir      The required direction for the GPIO pin.
 *          This can take one of the value from the following enum:
 *          - \ref GPIO_Direction_t.
 */
void GPIO_setDirMode(uint32_t baseAddr, uint32_t pinNum, uint32_t pinDir);

/**
 *  \brief  This API drives an output GPIO pin to a logic HIGH state.
 *
 *  \param  baseAddr    The memory address of the GPIO instance being used
 *  \param  pinNum      The number of the pin in the GPIO instance
 *          'pinNum' can take one of the following values:
 *          (0 <= pinNum <= #GPIO_MAX_PIN_PER_INSTANCE - 1)
 */
static inline void GPIO_pinWriteHigh(uint32_t baseAddr, uint32_t pinNum);

/**
 *  \brief  This API drives an output GPIO pin to a logic LOW state.
 *
 *  \param  baseAddr    The memory address of the GPIO instance being used
 *  \param  pinNum      The number of the pin in the GPIO instance
 *          'pinNum' can take one of the following values:
 *          (0 <= pinNum <= #GPIO_MAX_PIN_PER_INSTANCE - 1)
 */
static inline void GPIO_pinWriteLow(uint32_t baseAddr, uint32_t pinNum);

/**
 *  \brief  This API reads the logic level(value) on a specified
 *          GPIO pin.
 *
 *  \param  baseAddr    The memory address of the GPIO instance being used
 *  \param  pinNum      The number of the pin in the GPIO instance
 *          'pinNum' can take one of the following values:
 *          (0 <= pinNum <= #GPIO_MAX_PIN_PER_INSTANCE - 1)
 *
 *  \return  \ref GPIO_PinValue_t
 */
uint32_t GPIO_pinRead(uint32_t baseAddr, uint32_t pinNum);

/**
 *  \brief  This API determines the output logic level(value) on a specified
 *          GPIO pin.
 *
 *  \param  baseAddr    The memory address of the GPIO instance being used
 *  \param  pinNum      The number of the pin in the GPIO instance
 *          'pinNum' can take one of the following values:
 *          (0 <= pinNum <= #GPIO_MAX_PIN_PER_INSTANCE - 1)
 *
 *  \return \ref GPIO_PinValue_t
 */
uint32_t GPIO_pinOutValueRead(uint32_t baseAddr, uint32_t pinNum);

/**
 *  \brief  This API configures the trigger type for a specified input GPIO pin.
 *
 *          Whenever the selected trigger occurs on that GPIO pin and if interrupt
 *          generation is enabled for that pin, the GPIO module will send an
 *          interrupt to CPU.
 *          To disable the per pin event/interrupt generation, set the
 *          trigger type to #GPIO_TRIG_TYPE_NONE
 *
 *  \param  baseAddr    The memory address of the GPIO instance being used
 *  \param  pinNum      The number of the pin in the GPIO instance
 *          'pinNum' can take one of the following values:
 *          (0 <= pinNum <= 63).
 *  \param  trigType    This specifies the trigger type on whose detection,
 *                      the GPIO module will send an interrupt to CPU,
 *                      provided interrupt generation for that pin is enabled.
 *          'trigType' can take one of the following values following enum:
 *          - \ref GPIO_TrigType_t.
 */
void GPIO_setTrigType(uint32_t baseAddr, uint32_t pinNum, uint32_t trigType);

/**
 *  \brief  This API enables the bank interrupt. This has to be called after
 *          setting all the GPIO pin triggers of a bank to get interrupt
 *
 *  \param  baseAddr    The memory address of the GPIO instance being used
 *  \param  bankNum     The bank index
 *          'bankNum' can take one of the following values:
 *          (0 <= bankNum <= #GPIO_MAX_BANKS - 1)
 */
void GPIO_bankIntrEnable(uint32_t baseAddr, uint32_t bankNum);

/**
 *  \brief  This API disables the bank interrupt
 *
 *  \param  baseAddr    The memory address of the GPIO instance being used
 *  \param  bankNum     The bank index
 *          'bankNum' can take one of the following values:
 *          (0 <= bankNum <= #GPIO_MAX_BANKS - 1)
 */
void GPIO_bankIntrDisable(uint32_t baseAddr, uint32_t bankNum);

/**
 *  \brief  This API determines the enabled interrupt status of a specified pin.
 *
 *  \param  baseAddr    The memory address of the GPIO instance being used
 *  \param  pinNum      The number of the pin in the GPIO instance
 *          'pinNum' can take one of the following values:
 *          (0 <= pinNum <= #GPIO_MAX_PIN_PER_INSTANCE - 1)
 *
 *  \return intrStatus The enabled interrupt status of the pin on the specified
 *                     interrupt line.
 *                     1 - interrupt occurred
 *                     0 - interrupt hasnt occurred since last cleared
 *                         writing 1 clears the corresponding interrupt status
 */
static inline uint32_t GPIO_getIntrStatus(uint32_t baseAddr, uint32_t pinNum);

/**
 *  \brief  This API clears the enabled interrupt status of a specified GPIO
 *          pin.
 *
 *  \param  baseAddr    The memory address of the GPIO instance being used
 *  \param  pinNum      The number of the pin in the GPIO instance
 *          'pinNum' can take one of the following values:
 *          (0 <= pinNum <= #GPIO_MAX_PIN_PER_INSTANCE - 1)
 */
static inline void GPIO_clearIntrStatus(uint32_t baseAddr, uint32_t pinNum);

/**
 *  \brief  This API returns the interrupt status of the specified bank.
 *
 *  \param  baseAddr    The memory address of the GPIO instance being used
 *  \param  bankNum     Bank index of the GPIO instance.
 *
 *  \return Raw interrupt status for the provided register index
 *
 */
static inline uint32_t GPIO_getBankIntrStatus(uint32_t baseAddr, uint32_t bankNum);

/**
 *  \brief  This API clears the interrupt status of the specified bank.
 *
 *  \param  baseAddr    The memory address of the GPIO instance being used
 *  \param  bankNum     Bank index of the GPIO instance.
 *  \param  intrStatus  Interrupt mask to clear the status
 */
static inline void GPIO_clearBankIntrStatus(uint32_t baseAddr,
                                            uint32_t bankNum,
                                            uint32_t intrStatus);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

static inline void GPIO_pinWriteHigh(uint32_t baseAddr, uint32_t pinNum)
{
    uint32_t                regIndex, regVal;
    volatile CSL_GpioRegs*  hGpio = (volatile CSL_GpioRegs*)((uintptr_t) baseAddr);

    regIndex = GPIO_GET_REG_INDEX(pinNum);
    regVal = GPIO_GET_BIT_MASK(pinNum);
    CSL_REG32_WR(&hGpio->BANK_REGISTERS[regIndex].SET_DATA, regVal);

    return;
}

static inline void GPIO_pinWriteLow(uint32_t baseAddr, uint32_t pinNum)
{
    uint32_t                regIndex, regVal;
    volatile CSL_GpioRegs*  hGpio = (volatile CSL_GpioRegs*)((uintptr_t) baseAddr);

    regIndex = GPIO_GET_REG_INDEX(pinNum);
    regVal = GPIO_GET_BIT_MASK(pinNum);
    CSL_REG32_WR(&hGpio->BANK_REGISTERS[regIndex].CLR_DATA, regVal);

    return;
}

static inline uint32_t GPIO_getIntrStatus(uint32_t baseAddr, uint32_t pinNum)
{
    uint32_t                intrStatus;
    uint32_t                regIndex, bitPos;
    volatile CSL_GpioRegs*  hGpio = (volatile CSL_GpioRegs*)((uintptr_t) baseAddr);

    regIndex = GPIO_GET_REG_INDEX(pinNum);
    bitPos = GPIO_GET_BIT_POS(pinNum);
    intrStatus = CSL_FEXTR(hGpio->BANK_REGISTERS[regIndex].INTSTAT, bitPos, bitPos);

    return (intrStatus);
}

static inline void GPIO_clearIntrStatus(uint32_t baseAddr, uint32_t pinNum)
{
    uint32_t                regIndex, regVal;
    volatile CSL_GpioRegs*  hGpio = (volatile CSL_GpioRegs*)((uintptr_t) baseAddr);

    /* Clear the interrupt status of specified gpio pin */
    regIndex = GPIO_GET_REG_INDEX(pinNum);
    regVal = GPIO_GET_BIT_MASK(pinNum);
    CSL_REG32_WR(&hGpio->BANK_REGISTERS[regIndex].INTSTAT, regVal);

    return;
}

static inline uint32_t GPIO_getBankIntrStatus(uint32_t baseAddr, uint32_t bankNum)
{
    uint32_t                intrStatus, regIdx = bankNum >> 1U;
    volatile CSL_GpioRegs*  hGpio = (volatile CSL_GpioRegs*)((uintptr_t) baseAddr);

    intrStatus = CSL_REG32_RD(&hGpio->BANK_REGISTERS[regIdx].INTSTAT);
    if(bankNum & 0x01U)
    {
        intrStatus >>= GPIO_MAX_PIN_PER_BANK;   /* Odd number bank - upper 16-bits are used */
    }
    intrStatus &= 0xFFFFU;

    return (intrStatus);
}

static inline void GPIO_clearBankIntrStatus(uint32_t baseAddr,
                                            uint32_t bankNum,
                                            uint32_t intrStatus)
{
    uint32_t                regIdx = bankNum >> 1U;
    volatile CSL_GpioRegs*  hGpio = (volatile CSL_GpioRegs*)((uintptr_t) baseAddr);

    /* Clear the interrupt status of gpio bank */
    intrStatus &= 0xFFFFU;
    if(bankNum & 0x01U)
    {
        intrStatus <<= GPIO_MAX_PIN_PER_BANK;   /* Odd number bank - upper 16-bits are used */
    }
    CSL_REG32_WR(&hGpio->BANK_REGISTERS[regIdx].INTSTAT, intrStatus);

    return;
}

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef GPIO_V0_H_ */

/** @} */
