# LED {#BOARD_LED_PAGE}

[TOC]

The LED driver provides API to control I2C/GPIO based LED present in the board.
The driver supports multiple LED control for the LEDs connected to the same
I2C controller using a single API which can be used to set LED ON/OFF for multiple LED at the same time.

## Features Supported

- API to turn ON/OFF LED
- Set LED mask using single API to set the LED state of multiple LEDs connected
to same controller at one go

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

- Option to select LED type based on board
- Supported LED
    - GPIO
\cond SOC_AM64X || SOC_AM243X
    - TPIC2810 (I2C based)
    - IO Expander: TCA6424 (I2C based)
- Based on the type of LED controller selected, the GPIO or I2C driver can be configured
- In case of IO Expander based LED control, provides option to select the pin to which the LED is connected
- In case of I2C based LED controller, ability to provide the I2C target address of the controller
\endcond

## Features NOT Supported

- Set mask API is not supported for GPIO based LED as typically only one LED
is connected to a single instance of GPIO pin

## Important Usage Guidelines

None

## Example Usage

Include the below file to access the APIs
\snippet Led_sample.c include

LED ON API
\snippet Led_sample.c on

LED OFF API
\snippet Led_sample.c off

## API

\ref BOARD_LED_MODULE
