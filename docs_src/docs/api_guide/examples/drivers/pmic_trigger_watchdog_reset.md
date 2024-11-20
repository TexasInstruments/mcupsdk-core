# PMIC trigger watchdog reset example {#EXAMPLES_PMIC_TRIGGER_WATCHDOG_RESET}

[TOC]

# Introduction

The example demonstrates the PMIC watchdog in trigger mode which will generate a warm reset when the core fails to send the
trigger to the watchdog. The application configures the GPIO pin and the watchdog in the PMIC to receive the trigger signal.
It services the watchdog for few iterations and stops sending the trigger signal. Then, the core will wait for sometime
based on the reset threshold value configured in the watchdog for the warm reset to occur. If the warm reset occurs, the
test passes else fails.

\note This example won't work in the debug mode as the user must not service the watchdog while in debug mode. Flash the
application to get the expected result.

# Supported Combinations {#EXAMPLES_DRIVERS_PMIC_TRIGGER_WATCHDOG_RESET_COMBOS}

\cond SOC_AM261X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/pmic/pmic_trigger_watchdog_reset

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- As it is a watchdog example, it cannot be loaded through CCS as it may fail in debug mode. It should be flashed and booted
through SBL to get the expected results.
- Refer \ref GETTING_STARTED_FLASH for flashing the application.
- Set bootmode to  \ref BOOTMODE_UART and flash the binary using the following command from {SDK_INSTALLTION_PATH}/tools/boot
- Modify the existing default_sbl_ospi.cfg with the correct path of the application image.
    \code
    python uart_uniflash.py -p COM<x> --cfg=sbl_prebuilt/am261x-lp/default_sbl_ospi.cfg
    \endcode

- Power OFF and change the boot mode to \ref BOOTMODE_OSPI
- Logs should appear at the UART port

# Sample Output

Shown below is a sample output when the application is run,

\code
Starting trigger watchdog reset example !!
Checking if prerequisites for configuring the WDG has been met...
Configure PMIC watchdog in Trigger mode... DONE 
Started watchdog sequence... Sending watchdog trigger signal to the PMIC... 
Warm reset has occurred due to watchdog failure!! 
All tests have passed!!
\endcode

