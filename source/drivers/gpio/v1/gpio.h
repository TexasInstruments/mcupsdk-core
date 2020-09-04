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
 *  \file v1/gpio.h
 *
 *  \brief GPIO Driver API/interface file.
 *
 *  The GIO module supports up to eight 8-bit ports for a total of up to 64 GIO 
 *  terminals. Each of these 64 terminals can be independently configured as input
 *  or output.
 *
 */

#ifndef GPIO_H_
#define GPIO_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/cslr_gpio.h>
#include <kernel/dpl/SystemP.h>

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
 *   @{
 */
/** \brief Set GPIO direction as output */
#define GPIO_DIRECTION_OUTPUT           (1U)
/** \brief Set GPIO direction as input */
#define GPIO_DIRECTION_INPUT            (0U)
/** @} */

/**
 *  \anchor GPIO_OpenDrain_t
 *  \name GPIO OpenDrain
 *   @{
 */
/** \brief GPIO OpenDrain Enable */
#define GPIO_OPEN_DRAIN_ENABLE           (1U)
/** \brief GPIO OpenDrain Disable */
#define GPIO_OPEN_DRAIN_DISABLE          (0U)
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

/**
 *  \anchor GPIO_InterruptLevel_t
 *  \name GPIO Interrupt Level
 *  @{
 */
/** \brief GPIO low interrupt level */
#define GPIO_INTR_LEVEL_LOW             (0U)
/** \brief GPIO high interrupt level */
#define GPIO_INTR_LEVEL_HIGH            (1U)
/** @} */

/** \brief  This is the maximum number of ports */
#define GPIO_MAX_PORT                   (8U)

/**
* \brief
*  This is the maximum number of interrupt ports which can be supported.
*  Only Ports A, B, C and D are capable of interrupts. 
*/
#define GPIO_MAX_INTERRUPT_PORT         (4U)

/**
 * \brief
 *  This is the maximum number of pins per GPIO port
 *  This is a system limit on the GPIO IP Block.
 */
#define GPIO_MAX_PINS_PER_PORT          (8U)

/** \brief Maximum number of pins  */
#define GPIO_MAX_PINS                   (GPIO_MAX_PORT * GPIO_MAX_PINS_PER_PORT)

/**
 * \brief
 *  This is the maximum number of interrupts which can be supported.
 *  Only Ports A, B, C and D are capable of interrupts. Also since
 *  each port can have a maximum of 8 pins this is the maximum number
 *  of interrupts which can be supported
 */
#define GPIO_MAX_INTERRUPT              (GPIO_MAX_INTERRUPT_PORT * GPIO_MAX_PINS_PER_PORT)

/** \brief Returns the GPIO pin offset based on pin number */
#define GPIO_GET_PIN_OFFSET(pinNum)     (((uint32_t)pinNum) % (GPIO_MAX_PORT * GPIO_MAX_PINS_PER_PORT))
/** \brief Returns the GPIO port number based on pin number */
#define GPIO_GET_PORT_NUM(pinNum)       ((GPIO_GET_PIN_OFFSET(pinNum)) / GPIO_MAX_PINS_PER_PORT)
/** \brief Returns the pin Index based on pin number */
#define GPIO_GET_PIN_INDEX(pinNum)      ((GPIO_GET_PIN_OFFSET(pinNum)) % GPIO_MAX_PINS_PER_PORT)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * \brief  This API performs the module enable of the GPIO module.
 *
 * \param  baseAddr          GPIO base address
 *
 */
void GPIO_moduleEnable(uint32_t baseAddr);

/**
 * \brief  The function is used to configure the GPIO Pin to ignore 
 * or honor polarity.
 *
 * \param  baseAddr             GPIO base address
 * \param  pinNum               GPIO Pin number associated with the port
 *         'pinNum' can take one of the following values:
 *         (0 <= pinNum <= #GPIO_MAX_PINS - 1)
 * \param  polLevel             polarity of the interrupt to be triggered
 *         'polLevel' can take one of the following values following enum:
 *         - \ref GPIO_TrigType_t.
 *
 * \return  #SystemP_SUCCESS if PortNum is within range; else error on failure
 */
int32_t GPIO_ignoreOrHonorPolarity(uint32_t baseAddr, uint32_t pinNum, uint32_t polLevel);

/**
 * \brief  The function is used to configure the GPIO Pin to control
 *          the interrupt polarity
 *
 * \param  baseAddr        GPIO base address
 * \param  pinNum          GPIO Pin number associated with the port
 *         'pinNum' can take one of the following values:
 *         (0 <= pinNum <= #GPIO_MAX_PINS - 1)
 * \param  polLevel        polarity of the interrupt to be triggered
 *         'polLevel' can take one of the following values following enum:
 *         - \ref GPIO_TrigType_t.
 *
 * \return  #SystemP_SUCCESS if PortNum is within range; else error on failure
 */
int32_t GPIO_setTrigType(uint32_t baseAddr, uint32_t pinNum, uint32_t polLevel);

/**
 * \brief  The function is used to enable the interrupt on the GPIO Pin
 *
 * \param  baseAddr         GPIO base address
 * \param  pinNum           GPIO Pin number associated with the port
 *         'pinNum' can take one of the following values:
 *         (0 <= pinNum <= #GPIO_MAX_PINS - 1)
 *
 * \return  #SystemP_SUCCESS if PortNum is within range; else error on failure
 */
int32_t GPIO_enableInterrupt(uint32_t baseAddr, uint32_t pinNum);

/**
 * \brief  The function is used to disable the interrupt on the GPIO Pin
 *
 * \param  baseAddr         GPIO base address
 * \param  pinNum           GPIO Pin number associated with the port
 *         'pinNum' can take one of the following values:
 *         (0 <= pinNum <= #GPIO_MAX_PINS - 1)
 *
 * \return  #SystemP_SUCCESS if PortNum is within range; else error on failure
 */
int32_t GPIO_disableInterrupt(uint32_t baseAddr, uint32_t pinNum);


/**
 * \brief  The function is used to clear the interrupt on the GPIO Pin
 *
 * \param  baseAddr          GPIO base address
 * \param  pinNum            GPIO Pin number associated with the port
 *         'pinNum' can take one of the following values:
 *         (0 <= pinNum <= #GPIO_MAX_PINS - 1)
 *
 * \return #SystemP_SUCCESS if PortNum is within range; else error on failure
 */
int32_t GPIO_clearInterrupt(uint32_t baseAddr, uint32_t pinNum);

/**
 * \brief  The function is used to mark the interrupt as high level or low level
 *         for the specific GPIO Pin
 *
 * \param  baseAddr         GPIO base address
 * \param  pinNum           GPIO Pin number associated with the port
 *         'pinNum' can take one of the following values:
 *         (0 <= pinNum <= #GPIO_MAX_PINS - 1)
 * \param  interruptLevel   GPIO Interrupt priority
 *         'interruptLevel' can take one of the following values following enum:
 *         - \ref GPIO_InterruptLevel_t.
 *
 * \return #SystemP_SUCCESS if PortNum is within range; else error on failure
 */
int32_t GPIO_markHighLowLevelInterrupt(uint32_t baseAddr, uint32_t pinNum, uint32_t interruptLevel);

/**
 * \brief  The function is used to get the high level or low level interrupt pending.
 *         The driver will need to cycle through all the pending interrupts
 *         before the processing of the interrupt is over
 *
 * \param  baseAddr         GPIO base address
 * \param  interruptLevel   GPIO Interrupt priority
 *         'interruptLevel' can take one of the following values following enum:
 *         - \ref GPIO_InterruptLevel_t.
 *
 * \return pending interrupr number
 *         Zero        - No pending interrupt
 *         Non Zero    - Highest Pending Interrupt
 */
uint32_t GPIO_getHighLowLevelPendingInterrupt(uint32_t baseAddr, uint32_t interruptLevel);

/**
 * \brief  The function is used to set the output data direction
 *         associated with a GPIO Pin.
 *
 * \param  baseAddr         GPIO base address
 * \param  pinNum           GPIO Pin number associated with the port
 *         'pinNum' can take one of the following values:
 *         (0 <= pinNum <= #GPIO_MAX_PINS-1)
 * \param  pinDir           Pin Direction set
 *         This can take one of the value from the following enum:
 *         - \ref GPIO_Direction_t.
 *
 */
void GPIO_setDirMode(uint32_t baseAddr, uint32_t pinNum, uint32_t pinDir);

/**
 * \brief  The function is used to get the data input for a specific GPIO
 *         Port/Pin
 *
 * \param  baseAddr         GPIO base address
 * \param  pinNum           GPIO Pin number associated with the port
 *         'pinNum' can take one of the following values:
 *         (0 <= pinNum <= #GPIO_MAX_PINS-1)
 *
 * \return  \ref GPIO_PinValue_t
 */
uint32_t GPIO_pinRead(uint32_t baseAddr, uint32_t pinNum);

/**
 * \brief  The function is used to write logic HIGH state to a  to a specific GPIO Pin
 *
 * \param  baseAddr         GPIO base address
 * \param  pinNum           GPIO Pin number associated with the port
 *         'pinNum' can take one of the following values:
 *         (0 <= pinNum <= #GPIO_MAX_PINS-1)
 *
 */
void GPIO_pinWriteHigh(uint32_t baseAddr, uint32_t pinNum);


/**
 * \brief  The function is used to write logic LOW state to a specific GPIO Port/Pin
 *
 * \param  baseAddr         GPIO base address
 * \param  pinNum           GPIO Pin number associated with the port
 *         'pin' can take one of the following values:
 *         (0 <= pin <= #GPIO_MAX_PINS-1)
 *
 */
void GPIO_pinWriteLow(uint32_t baseAddr, uint32_t pinNum);


/**
 * \brief  The function is used to enable/disable open drain functionality
 *         for a specific GPIO Port/Pin. This function is applicable for GPIO pins
 *         which are configured as output.
 *
 * \param  baseAddr         GPIO base address
 * \param  pinNum           GPIO Pin number associated with the port
 *         'pinNum' can take one of the following values:
 *         (0 <= pinNum <= #GPIO_MAX_PINS-1)
 * \param  openDrainStatus  Set open drain status
 *         This can take one of the value from the following enum:
 *         - \ref GPIO_OpenDrain_t.
 *
 */
void GPIO_setOpenDrainStatus(uint32_t baseAddr, uint32_t pinNum, uint32_t openDrainStatus);

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef GPIO_H_ */

/** @} */
