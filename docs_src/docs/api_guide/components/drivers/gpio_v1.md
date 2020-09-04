# GPIO {#DRIVERS_GPIO_PAGE}

[TOC]

The General-Purpose Input/Output (GPIO) driver provides API to configure general-purpose pins as either inputs or outputs.
It also provided API to configure GPIO to produce host CPU interrupts and DMA synchronization events in different
interrupt/event generation modes.

## Features Supported

- Supports up to 8-bit ports for a total of 64 GPIO
- Supports up to 32 GPIO's which can be configured as interrupt
- Supports up to 32 interrupt generating GPIO's.
- Set/clear functionality

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

- Set pin direction: input or output
- Set interrupt trigger type
- Set openDrain: enable or disable
- Set interrupt level: high or low
- Configuring pinmux based on selected pin

## Features NOT Supported

NA

## Important Usage Guidelines

- Note: Not all GPIO pins, are present in a particular device.
Refer device TRM for actual GPIO instances and pins supported

## Example Usage

Include the below file to access the APIs
\snippet Gpio_sample_v1.c include

GPIO configuration as output
\snippet Gpio_sample_v1.c output

GPIO configuration as input
\snippet Gpio_sample_v1.c input

GPIO configuration for interrupt
\snippet Gpio_sample_v1.c interrupt

## API

\ref DRV_GPIO_MODULE
