# Introduction {#mainpage}

[TOC]

Welcome to **@VAR_SDK_NAME for @VAR_SOC_NAME**. This SDK contains examples, libraries and tools to develop **RTOS and no-RTOS** based applications for **ARM R5F CPU** and related peripherals.

## Getting Started

To get started, see \ref GETTING_STARTED

## Block Diagram

Given below is a block diagram of the SW modules in this SDK,

\imageStyle{block_diagram_am263px.png,width:70%}
\image html block_diagram_am263px.png "Software Block Diagram"

The main software components in the block diagram are described below


<table>
<tr>
    <th>Software Components
    <th>Documentation Page
    <th>Description
</tr>
<tr><td colspan="3" bgcolor=#F0F0F0><b>OS Kernel</b></td></tr>
<tr>
    <td>No RTOS
    <td> \ref KERNEL_NORTOS_PAGE
    <td>Contains modules which implement no-RTOS execution environment consisting of timers, ISR, main thread.
    Allows software on top to run in bare metal mode.</td>
</tr>
<tr>
    <td>FreeRTOS Kernel
    <td> \ref KERNEL_FREERTOS_PAGE
    <td>FreeRTOS Kernel, provides a execution environment consisting of tasks, semaphores, timers, see https://www.freertos.org/RTOS.html
</tr>
<tr>
    <td>FreeRTOS POSIX
    <td> \ref KERNEL_FREERTOS_PAGE
    <td>Limited POSIX APIs with FreeRTOS underneath, provides pthreads, mqueue, semaphore, see https://www.freertos.org/FreeRTOS-Plus/FreeRTOS_Plus_POSIX/index.html
</tr>
<tr>
    <td>Driver Porting Layer (DPL)
    <td> \ref KERNEL_DPL_PAGE
    <td>APIs used by drivers to abstract the OS environment. Example, Semaphore, HW interrupts, mutex, clock.
</tr>
<tr><td colspan="3" bgcolor=#F0F0F0><b>Device Drivers and HAL (Hardware Abstraction Layer)</b></td></tr>
<tr>
    <td>SOC Peripheral Drivers
    <td>\ref DRIVERS_PAGE
    <td>Device Drivers library and APIs for peripherals within the SOC. Example, I2C, GPIO, UART.
</tr>
<tr>
    <td>Board Peripheral Drivers
    <td>\ref BOARD_DRIVERS_PAGE
    <td>Device Drivers library and APIs for peripherals on the board or EVM. Example, Flash, EEPROM.
</tr>
<tr><td colspan="3" bgcolor=#F0F0F0><b>Protocol Stacks and Middleware</b></td></tr>
<tr>
    <td>Mbed-TLS
    <td>\ref EXAMPLES_CPSW_LWIP_HTTPS
    <td>TLS and SSL protocol implementation with respective cryptographic algorithm support
</tr>
<tr><td colspan="3" bgcolor=#F0F0F0><b>Software Diagnostics Library</b></td></tr>
<tr>
    <td>SDL
    <td>\ref SDL_PAGE
    <td>Software Diagnostics Libaray
</tr>
</tr>
<tr><td colspan="3" bgcolor=#F0F0F0><b>Examples and Demos</b></td></tr>
<tr>
    <td>Examples and Demos
    <td>\ref EXAMPLES
    <td>Examples and demos showing usage of different SW libraries and APIs
</tr>
<tr><td colspan="3" bgcolor=#F0F0F0><b>Tools (used on host machine)</b></td></tr>
<tr>
    <td> Smart Placement
    <td>\ref SMART_PLACEMENT
    <td> Smart placement is a process which aims to place critical functions & data in faster memory and is usually used for large memory footprint application which requires XIP.
</tr>
<tr>
    <td> Memory Configurator
    <td>\ref MEMORY_CONFIGURATOR
    <td> Memory Configurator is a tool which provides option to choose critical parameters to generate linker files for a seamless, hassle-free experience.
</tr>
<tr>
    <td>Code Composer Studio (CCS)
    <td>\ref CCS_PROJECTS_PAGE
    <td>IDE used to build projects, debug programs, see http://software-dl.ti.com/ccs/esd/documents/users_guide/index_project-management.html
</tr>
<tr>
    <td>TI CLANG Compiler Toolchain
    <td>\htmllink{https://www.ti.com/tool/download/ARM-CGT-CLANG-1, **TI CLANG CGT HOMEPAGE**}
    <td>CLANG based ARM compiler from TI for ARM R5F and M4F
</tr>
<tr>
    <td>SysConfig
    <td>\ref SYSCONFIG_INTRO_PAGE
    <td>System configuration tool, used to configure peripherals, pinmux, clocks and generate system initialization code
</tr>
<tr>
    <td>TI Resource Explorer (TIREX)
    <td>\ref TIREX_INTRO_PAGE
    <td>Web broswer based tool to explore the SDK, select, import and run the examples
</tr>
<tr>
    <td>SDK Tools and Utilities
    <td> \ref TOOLS
    <td>Additional tools and utilities, like flashing tools, booting tools, CCS loading scripts used with the SDK development flow
</tr>
</table>

## Directory Structure

Given below is a overview of the directory structure to help you navigate the SDK and related tools.

<table>
<tr>
    <th>Folder/Files
    <th>Description
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/</td></tr>
<tr>
    <td>README_FIRST_@VAR_SOC_NAME.html
    <td>Open this file in a web browser to reach this user guide</td>
</tr>
<tr>
    <td>makefile
    <td>Top level makefile to build the whole SDK using "make"</td>
</tr>
<tr>
    <td>imports.mak
    <td>Top level makefile to list paths to dependent tools</td>
</tr>
<tr>
    <td>docs/
    <td>Offline browseable HTML documentation</td>
</tr>
<tr>
    <td>examples/
    <td>Example applications for @VAR_SOC_NAME, across multiple boards, CPUs, NO-RTOS, RTOS</td>
</tr>
<tr>
    <td>source/
    <td>Device drivers, middleware libraries and APIs</td>
</tr>
<tr>
    <td>tools/
    <td>Tools and utilities like CCS loading scripts, initialization scripts.
    </td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/source/</td></tr>
<tr>
    <td>board/
    <td>Board peripheral device drivers</td>
</tr>
<tr>
    <td>drivers/
    <td>SOC peripheral device drivers</td>
</tr>
<tr>
    <td>kernel/
    <td>NO RTOS and RTOS kernel and Driver Porting layer (DPL) for these environments</td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/examples/</td></tr>
<tr>
    <td>drivers/
    <td>SOC and board focused device driver examples. The examples are based on both NO-RTOS and RTOS. </td>
</tr>
<tr>
    <td>empty/
    <td>Template projects to copy into workspace and customize based on application needs</td>
</tr>
</table>

Given below are the paths where the different tools needed outside the SDK, like CCS, SysConfig are installed by default in Windows.
In Linux, the tools are installed by default in ${HOME}/ti.

<table>
<tr>
    <th>Folder/Files
    <th>Description
</tr>
<tr>
    <td>C:/ti/ccs@VAR_CCS_FOLDER_VERSION
    <td>Code Composer Studio</td>
</tr>
<tr>
    <td>C:/ti/sysconfig_@VAR_SYSCFG_VERSION
    <td>SysConfig. **NOTE**, SysConfig is also installed as part of CCS at ${CCS_INSTALL_PATH}/ccs/utils/sysconfig_x.x.x</td>
</tr>
<tr>
    <td>C:/ti/ti-cgt-armllvm_@VAR_TI_ARM_CLANG_VERSION
    <td>TI ARM CLANG compiler tool chain</td>
</tr>
</table>

## Licenses

The licensing information of this SDK, as well as any third-party components included which are made available under a number of other open-source licenses are enumerated as part of the manifest.
A complete manifest along with export control information is detailed here [\htmllink{../../docs/@VAR_SOC_MANIFEST,LINK}], [\htmllink{../../docs/mcu_plus_sdk_tsn_manifest.html,LINK}] and the SDK Software License Agreement (SLA) is here [\htmllink{../../license.txt,LINK}]

## Help and Support

For additional help and support, see https://e2e.ti.com/support/microcontrollers/arm-based-microcontrollers-group/arm-based-microcontrollers/f/arm-based-microcontrollers-forum

## Documentation Credits

This user guide is generated using doxygen, v1.8.20. See https://www.doxygen.nl/index.html