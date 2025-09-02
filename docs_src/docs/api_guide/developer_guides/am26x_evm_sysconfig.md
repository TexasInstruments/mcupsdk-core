# Enabling AM26x EVM Configuration Support in SysConfig {#EVM_SYSCONFIG_GUIDE}

[TOC]

## Introduction

The SysConfig GUI tool is used to configure SDK examples at the device level, and can now be used to configure SDK examples at the board level.
By using SysConfig with TI AM26x EVMs, external header signals and on-board peripheral hardware can be configured in real-time and initialized through generated SysConfig code.
AM26x EVM support in SysConfig aims to reduce bring-up and development time using TI AM26x EVMs by including many of the details typically found in the online User Guide directly in the SysConfig GUI.
This guide walks through how to enable the AM26x EVM support within SysConfig.

\note AM26x EVM support in SysConfig requires the MCU+ SDK v11.0 and SysConfig 1.24.2 or later.

### EVM Features Enabled by SysConfig

In this SDK, SysConfig is used to configure the following on EVMs:
- On-board peripheral hardware such as I2C Buses, Headers, Transceivers, LEDs
- External signal header pinouts for the following TI Standard EVMs:
    - LaunchPad: BoosterPack Header
    - controlCARD: HSEC Connector
    - SOM: High-Density Connector
- Relevant software drivers for on-board signals

SysConfig is also used to showcase elements of the User Guide that are useful in configuring the EVM, such as:
- Switch positions
- Push button functionality
- On-board hardware specifications

## Enabling EVM Support in SysConfig - Standalone 
\note To view the EVM in SysConfig, the MCU+ SDK v11.0 and SysConfig 1.24.2 or later are required.

Start by opening SysConfig. Once the Software Product is selected, an additional 'Board' tab will appear.

Select the desired EVM (AM261x LaunchPad will be used for this guide) and click START.

\image html sysconfig_start_screen.png

### Configuring on-board Peripheral Hardware
Once inside SysConfig, navigate to the Hardware tab to view all on-board configurable hardware.

\image html hardware_tab.png

In the sidebar, on-board configurable hardware is sorted by peripheral and hardware type. Note that not all hardware is supported by software configuration using the SysConfig GUI, such as switches and buttons that must be physically manipulated. 

To add EVM hardware to a project design, click the '+' next to the component, or click on a component to open its full description. The I2C0_BUS is shown as an example:

\image html configuring_hardware.png

The Description window gives information about the component that would typically be found in the EVM User Guide. Once Use Software is selected, the full component driver configuration window opens, allowing direct configuration of the on-board peripheral:

\image html add_software.png

The untitled.syscfg file in the Generated Files window will automatically update code based on selections made in the GUI. This file is compiled as part of the binary file loaded onto the MCU to configure the device pins and on-board hardware.

\image html generating_code.png

### Enabling External Header Signals
SysConfig can be used to add specific signals to their designated external header pin. Standard TI EVMs, such as LaunchPads, controlCARDs, and SOMs have standard locations on their external header interfaces for typical MCU signals, allowing for universal support of accessory boards and docking hardware.

In SysConfig, locate the external header interface drop-down (BoosterPack, HSEC, SOM) to see all available signals on the external header.

\image html ext_header_dropdown.png

Peripheral signals are added in the same way as on-board hardware. Using EPWM as an example, select the peripheral to open up the description window. Click '+' or Use Software to add the peripheral signal(s) to the design. The software driver configuration window will open. On the right, a GUI showing all of the EVM headers will dynamically change depending on what signals are selected.

\image html header_signal_add.png

### Enabling Header Signals via PinMux
Peripheral components not defined in the Hardware tab can still be added to an EVM project design. In the Reserve Peripherals or Software tab, signals can be added to a design to be brought out to external headers on an EVM.

Hovering over any pin in the GUI shows all available signals via the device PinMux on any given external header. For example, hovering over BoosterPack pin 3 in the GUI for AM261x LaunchPad shows that while the pin is defined for UART3_RXD, several other signals can be accessed on this pin through the device PinMux.

\image html pinmux_gui.png

MuxMode 7 on this pin indicates that GPIO119 is accessible on this pin. In the Software tab, navigate to /drivers/gpio/gpio.

\image html gpio_driver.png

Notice that when adding GPIO119, a warning will appear. This is to alert the user that this pin is connected to on-board hardware, and can be suppressed. The EVM header GUI will change to indicate that this pin is being configured by software.

\image html gpio_driver_config.png

GPIO119 is now shown as 'Used' on the header GUI:

\image html gpio_selected_gui.png

## Non-Configurable Interfaces

As previously mentioned, there are some on-board interfaces that cannot be configured directly in the Hardware tab of SysConfig:
- Physical switches
- Push buttons
- Ethernet PHYs (configurable in Software tab)
- Flash memory devices (configurable in Software tab)
- PMICs (configurable in Software tab)

## SDK Example

- Please refer to the example \ref EXAMPLES_HELLO_WORLD_BOARD_SYSCFG that has the EVM Config support feature enabled.
