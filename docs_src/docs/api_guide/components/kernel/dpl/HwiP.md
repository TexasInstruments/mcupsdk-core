# HW Interrupts {#KERNEL_DPL_HWI_PAGE}

[TOC]

\attention See also \ref KERNEL_FREERTOS_PAGE, \ref KERNEL_NORTOS_PAGE for list of CPU specific supported and unsupported features.

## Features Supported

- Register a interrupt callback to a specific CPU number
- Ability to pass user specific argument to the interrupt callback
- Enable, disable, restore and clear specific CPU interrupts
- Enable, disable, restore global CPU interrupt
\cond !SOC_AM62X
- For ARM R5,
  - Ability to specify interrupt as FIQ or IRQ, level or pulse
  - Ability to specify interrupt priority
\endcond
- For ARM M4,
  - Ability to specify interrupt priority
  - Ability to specify systick ISR and NVIC external interrupt ISR
\cond SOC_AM64X
- For ARM A53,
  - Ability to specify interrupt priority
  - Nested Interrupts
  - Ability to specify interrupt as level or pulse
\endcond

## Features NOT Supported

See also \ref KERNEL_FREERTOS_PAGE, \ref KERNEL_NORTOS_PAGE for list of unsupported features.

## Important Usage Guidelines

\cond !SOC_AM62X
- For ARM R5,
  - TI VIM is the interrupt controller that is supported.
  - \ref HwiP_disable, \ref HwiP_restore, \ref HwiP_enable only affect state of IRQ. FIQ state is not changed
  - Refer ARMv7-R Architecture reference manual and SOC TRM for more details.
\endcond
- For ARM M4,
  - ARM NVIC is the interrupt controller that is supported.
  - Interrupt numbers 0 to 15 are for internal interrupts,
    like reset (1), NMI (2), fault handlers (3-6), SVC (11), PendSV (14), SysTick (15)
  - Interrupt numbers 16 to 80 are used as external NVIC interrupts.
    The TRM will document M4F interrupt numbers as xxx_M4FSSx_COREx_NVIC_IN_n.
    This corresponds to interrupt number (16 + n) at NVIC and (16 + n) is used as input to the HwiP APIs
  - Refer ARMv7-M Architecture reference manual and SOC TRM for more details.
\cond SOC_AM64X
- For ARM A53,
  - GIC V3 is the interrupt controller that is supported.
  - Interrupt numbers 0-15 are for SGI, 16-31 for PPI and 32-255 for SPI.
  - Refer ARMv8-A Architecture reference manual and SOC TRM for more details.
\endcond
\cond SOC_AM243X
- On @VAR_SOC_NAME,
    CPU type  | Valid interrupt numbers  | Valid interrupt priorities
    ----------|--------------------------|---------------------------
    R5F       | 0  .. 511                | 0 (highest) .. 15 (lowest)
    M4F       | 15 ..  79                | 0 (highest) ..  7 (lowest)
\endcond

\cond SOC_AM64X
- On @VAR_SOC_NAME,
    CPU type  | Valid interrupt numbers  | Valid interrupt priorities
    ----------|--------------------------|---------------------------
    R5F       | 0  .. 511                | 0 (highest) .. 15 (lowest)
    M4F       | 15 ..  79                | 0 (highest) ..  7 (lowest)
    A53       | 0 ..  255                | 0 (highest) ..  15 (lowest)
\endcond

\cond SOC_AM62X
- On @VAR_SOC_NAME,
    CPU type  | Valid interrupt numbers  | Valid interrupt priorities
    ----------|--------------------------|---------------------------
    M4F       | 15 ..  79                | 0 (highest) ..  7 (lowest)
\endcond

## Example Usage

Include the below file to access the APIs,
\snippet HwiP_sample.c include

Example ISR,
\snippet HwiP_sample.c isr

\cond !SOC_AM62X
Example to register a ISR for CPU interrupt 10,
\snippet HwiP_sample.c register
\endcond

\cond SOC_AM62X
Example to register a ISR for CPU interrupt 10,
\snippet HwiP_m4_sample.c register
\endcond

Example to disable and restore interrupts across a crtical section
\snippet HwiP_sample.c disable

## API

\ref KERNEL_DPL_HWI