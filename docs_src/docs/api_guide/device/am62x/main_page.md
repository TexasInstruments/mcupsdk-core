# Introduction {#mainpage}

[TOC]

Welcome to **@VAR_SDK_NAME for @VAR_SOC_NAME**. This SDK contains examples, libraries and tools to develop **RTOS and no-RTOS** based applications for **ARM M4F CPU** and related peripherals. Please note that MCU+SDK for @VAR_SOC_NAME only supports M4F MCU.
This package should be used along with PROCESSOR-SDK-LINUX-AM62X 08.03.00.

## Getting Started

To get started, see \ref GETTING_STARTED

## Block Diagram

Given below is a block diagram of the SW modules in this SDK,

\imageStyle{block_diagram_am62x.png,width:70%}
\image html block_diagram_am62x.png "Software Block Diagram"

The main software components in the block diagram are described below


<table>
<tr>
    <th>Software Components
    <th>Documentation Page
    <th>Description
</tr>
<tr><td colspan="3" bgcolor=#F0F0F0>**OS Kernel**</td></tr>
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
    <td>Driver Porting Layer (DPL)
    <td> \ref KERNEL_DPL_PAGE
    <td>APIs used by drivers to abstract the OS environment. Example, Semaphore, HW interrupts, mutex, clock.
</tr>
<tr><td colspan="3" bgcolor=#F0F0F0>**Device Drivers and HAL (Hardware Abstraction Layer)**</td></tr>
<tr>
    <td>SOC Peripheral Drivers
    <td>\ref DRIVERS_PAGE
    <td>Device Drivers library and APIs for peripherals within the SOC. Currenlty only IPC_RPMessage between A53 and M4 is supported. More drivers will be added in the future.
</tr>
<tr><td colspan="3" bgcolor=#F0F0F0>**Examples and Demos**</td></tr>
<tr>
    <td>Examples and Demos
    <td>\ref EXAMPLES
    <td>Examples and demos showing usage of different SW libraries and APIs
</tr>
<tr><td colspan="3" bgcolor=#F0F0F0>**Tools (used on host machine)**</td></tr>
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
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/source/</td></tr>
<tr>
    <td>board/
    <td>Board peripheral device drivers. Currently no board peripheral drivers are supported. More drivers will be added in the future.</td>
</tr>
<tr>
    <td>drivers/
    <td>Soc peripheral device drivers. Currently only IPC_RPMessage supported. More drivers will be added in the future.</td>
</tr>
<tr>
    <td>kernel/
    <td>NO RTOS and RTOS kernel and Driver Porting layer (DPL) for these environments.</td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/examples/</td></tr>
<tr>
    <td>drivers/
    <td>SOC and board level focused device drivers examples. The examples are based on both NO-RTOS and RTOS </td>
</tr>
<tr>
    <td>kernel/
    <td>NO RTOS and RTOS kernel focused examples</td>
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
A complete manifest along with export control information is detailed here [\htmllink{../../docs/@VAR_SOC_MANIFEST,LINK}] and the SDK Software License Agreement (SLA) is here [\htmllink{../../license.txt,LINK}]

## Help and Support

For additional help and support, see https://e2e.ti.com/support/processors/f/processors-forum

## Documentation Credits

This user guide is generated using doxygen, v1.8.20. See https://www.doxygen.nl/index.html