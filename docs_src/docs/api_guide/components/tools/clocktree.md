#  Clocktree {#CLOCKTREE}

[TOC]

## Introduction

Previously, the MCU+SDK offered limited flexibility in selecting clock sources or frequencies through the sysconfig tool, with most values hardcoded for various modules. These hardcoded values were reflected in the ti_power_clock_config.c file, which is one of the generated sysconfig files. Additionally, the clock values for the PLLs were also hardcoded within the soc_bootloader.c files.

With the introduction of the clock tree, users now have an enhanced view of the entire clock architecture, illustrating how clock sources, multiplexers, dividers, and peripherals are interconnected. This centralizes the configurability of the clocktree, allowing users to adjust these values directly from the ClockTree view, with the changes appearing as read-only values in the Software view. This update significantly reduces the hardcoding of clock values that was prevalent in earlier versions. However, some combinations of muxes and dividers may still produce incompatible input clock frequencies for certain peripherals, and the system includes errors and warnings to address these scenarios.

This new approach also minimizes human errors. For instance, when two modules share the same muxes (such as UART and LIN), any modifications made will automatically reflect on both modules, ensuring consistency that could otherwise be compromised with manual configurations.

## Overview

In the ClockTree panel view, components are organized into four categories:

- CLOCKS
- PLLs
- XBARs
- PERIPHERALS

Most configurations will focus on either the PLLs or the PERIPHERALS. The PLL view may appear complex, as it encompasses the entire clocktree architecture. Therefore, it is advisable for users to configure settings from the PERIPHERAL view if they know which peripheral is connected to the PLL they wish to modify.

\imageStyle{clocksource_to_peripheral.png,width:50%}
    \image html clocksource_to_peripheral.png "How a peripheral derives its input clock from Clock source"

Since all the components from the clock sources, PLLs, DPLLs etc. are interconnected, any changes made to shared components will be automatically synchronized across all dependent modules.

Modules such as UART, I2C, LIN and a few others previously allowed configuration of clock sources and frequencies from the Software View. Now, all configurations must be made through the ClockTree, with any changes reflected as read-only values in the Software View:

\imageStyle{uart_clock_arch.png,width:50%}
    \image html uart_clock_arch.png "UART Clocktree"

\imageStyle{uart_sw_view.png,width:50%}
    \image html uart_sw_view.png "UART Clock source and input clock information coming from Clocktree"

\imageStyle{uart_generated_code.png,width:50%}
    \image html uart_generated_code.png "UART Clock configuration values getting generated"


Subsequently, the ti_power_clock_config.c file will be generated with these updated values.

PLL configuration is also possible; however, for changes to take effect, the example must include the Bootloader module added from sysconfig. This will generate two files: ti_clocktree_pll_config.h and ti_clocktree_pll_config.c, with the latter containing the updated PLL configuration.

\imageStyle{pll_files_generated.png,width:50%}
    \image html pll_files_generated.png "PLL Files getting generated from Sysconfig"

\imageStyle{pll_files_generated.png,width:50%}
    \image html pll_files_generated.png "PLL clock frequencies change from Clocktree"

\cond SOC_AM261X

## Switching between 500 MHz and 400 MHz in ZFG package {#CLOCKTREE_VARIANT_SWITCHING}

For the AM261x ZFG Package:
There are two variants available, catering to different R5F clock frequencies: 400 MHz and 500 MHz. The default variant is 500 MHz, but users can switch to the 400 MHz variant. The pinmux configuration will remain the same as ZFG, but the clocktree architecture will align with that of ZCZ.

\imageStyle{am261x_zfg_variant_switch.png,width:50%}
    \image html am261x_zfg_variant_switch.png "Navigate to the Device View to Switch between Variants"

\imageStyle{am261x_zfg_variant_change_variant.png,width:50%}
    \image html am261x_zfg_variant_change_variant.png "Switching between Variants"

\endcond