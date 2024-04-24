/*
 * Copyright (C) 2024 Texas Instruments Incorporated
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

#ifndef GPIO_CONTROLLER_H_
#define GPIO_CONTROLLER_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/gpio.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "../../gpio_controller_mcspi_peripheral.h"

/* ========================================================================== */
/*                             Typedefs & Macros                              */
/* ========================================================================== */
#define REG_MASK           (0x1F)
/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* Structure to control the action on the GPIO pin */
typedef struct {
    volatile uint32_t *set;
    volatile uint32_t *clr;
    volatile uint32_t *in;
    uint32_t regMask;
} SpiPinControl;

/* Structure to hold all gpio pins used for SPI transfer */
typedef struct {
    SpiPinControl spiclk;
    SpiPinControl mosi;
    SpiPinControl miso;
    SpiPinControl cs;
} SpiGpioControl;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* This function initializes the GPIO pins */
static inline void App_initSpiGpioControl(SpiGpioControl *);

/* This function emulates the SPI master */
static inline void App_spiGpioTxRx();

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

static inline void App_initSpiGpioControl(SpiGpioControl *controller)
{
    controller->spiclk.set = (volatile uint32_t *)&((volatile CSL_GpioRegs*)((uintptr_t)SPICLK_BASE_ADDR))->BANK_REGISTERS[SPICLK_PIN >> GPIO_PINS_PER_REG_SHIFT].SET_DATA; /* Directly write to the SET_DATA register (pinHigh) */
    controller->spiclk.clr = (volatile uint32_t *)&((volatile CSL_GpioRegs*)((uintptr_t)SPICLK_BASE_ADDR))->BANK_REGISTERS[SPICLK_PIN >> GPIO_PINS_PER_REG_SHIFT].CLR_DATA; /* Directly write to the CLR_DATA register (pinLow) */
    controller->spiclk.regMask = 1U << (SPICLK_PIN & REG_MASK);

    controller->mosi.set = (volatile uint32_t *)&((volatile CSL_GpioRegs*)((uintptr_t)MOSI_BASE_ADDR))->BANK_REGISTERS[MOSI_PIN >> GPIO_PINS_PER_REG_SHIFT].SET_DATA;
    controller->mosi.clr = (volatile uint32_t *)&((volatile CSL_GpioRegs*)((uintptr_t)MOSI_BASE_ADDR))->BANK_REGISTERS[MOSI_PIN >> GPIO_PINS_PER_REG_SHIFT].CLR_DATA;
    controller->mosi.regMask = 1U << (MOSI_PIN & REG_MASK);

    controller->cs.set = (volatile uint32_t *)&((volatile CSL_GpioRegs*)((uintptr_t)CS_BASE_ADDR))->BANK_REGISTERS[CS_PIN >> GPIO_PINS_PER_REG_SHIFT].SET_DATA;
    controller->cs.clr = (volatile uint32_t *)&((volatile CSL_GpioRegs*)((uintptr_t)CS_BASE_ADDR))->BANK_REGISTERS[CS_PIN >> GPIO_PINS_PER_REG_SHIFT].CLR_DATA;
    controller->cs.regMask = 1U << (CS_PIN & REG_MASK);

    controller->miso.in = (volatile uint32_t *)&((volatile CSL_GpioRegs*)((uintptr_t)MISO_BASE_ADDR))->BANK_REGISTERS[MISO_PIN >> GPIO_PINS_PER_REG_SHIFT].IN_DATA; /* Directly Read from IN_DATA register (pinRead) */
    controller->miso.regMask = 1U << (MISO_PIN & REG_MASK);
}

static inline void App_spiGpioTxRx()
{
        SpiGpioControl controller; /* Creating a local instance of the controller struct for better performance */
        App_initSpiGpioControl(&controller); /* Initialising the struct */

        for(int loop = 0; loop < APP_GPIOSPI_MSGSIZE; ++loop)
		{
#ifdef APP_GPIOSPI_8BIT
            uint8_t tx = gMcspiTxBuffer[loop];
            uint8_t rx = 0;
#else
            uint32_t tx = gMcspiTxBuffer[loop];
            uint32_t rx = 0;
#endif

            *controller.cs.clr = controller.cs.regMask; /* Select Peripheral */

            for(int k = 0; k < APP_GPIOSPI_DATASIZE; ++k)
			{

                if(tx & MASK)
				{
                    *controller.mosi.set = controller.mosi.regMask; /* Send '1' */
                }
                else
				{
                    *controller.mosi.clr = controller.mosi.regMask; /* Send '0 */
                }

                *controller.spiclk.set = controller.spiclk.regMask; /* SPICLK HIGH */

                rx = (rx << 1) | ((*controller.miso.in & controller.miso.regMask) != 0); /* Read MISO line and append to rx */

                *controller.spiclk.clr = controller.spiclk.regMask; /* SPICLK LOW */

                tx <<= 1;
            }

            *controller.cs.set = controller.cs.regMask; /* De-select Peripheral */

            gMcspiRxBuffer[loop] = rx;
        }
}

#endif