# GPTIMER {#DRIVERS_GPTIMER_PAGE}

[TOC]

The General-Purpose Timer driver provides APIs to configure the available timer modules
in multiple modes.

## Features Supported

- Interrupts generated on overflow, compare, and capture
- Free-running 32-bit upward counter
- Compare and capture modes
- Autoreload mode
- Start/stop mode
- Programmable divider clock source (2 exponent n, where n = [0-8])
- Dedicated input trigger for capture mode and dedicated output trigger/PWM signal
- On-the-fly read/write register

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

- Set mode of operation of the timer
- Enable/Disable and Set counter Prescaler
- Enable/Disable Auto reload Mode and set Auto reload Value
- Enable Interrupt and Assign User Callback
- Set Compare value when Compare Match mode is set
- Set Counter mode and Capture event when Timer configured to capture input
- Set PWM output Trigger mode and Compare Value in PWM mode
- Set Default Output Polarity in PWM mode
- Set Output Pin Modulation in PWM mode

## Features NOT Supported

- Atomic 64-bit timer value read of cascaded timers
- Smart-idle with wake-up mode

## Example Usage

Include the below file to access the APIs
\snippet Gp_timer_sample.c include

Instance Open Example
\snippet Gp_timer_sample.c open

Instance Close Example
\snippet Gp_timer_sample.c close

GPTIMER configuration in Free Running Mode without interrupt
\snippet Gp_timer_sample.c free_run_no_interrupt

GPTIMER configuration in Free Running Mode with interrupt
\snippet Gp_timer_sample.c free_run_interrupt

GPTIMER configuration in Output Compare Mode
\snippet Gp_timer_sample.c output_compare_interrupt

GPTIMER configuration in Input Capture Mode
\snippet Gp_timer_sample.c input_capture_interrupt

GPTIMER configuration in PWM Generation Mode
\snippet Gp_timer_sample.c pwm_gen

## API

\ref DRV_GPTIMER_MODULE
