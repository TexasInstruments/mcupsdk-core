# ADS85x8 {#EXAMPLES_PRU_ADC_ADS85x8}

[TOC]

## Introduction

This example uses ADC ICs from the below list to convert analog input signal on adc channels to samples and store the results in a shared memory which can be configured by the user (using \ref DRIVERS_PRU_IPC module). The read samples are finally printed to the R5F console, or can also be visualized using CCS Graph plotter. GUI Composer app available on TI Gallery can also be used to evaluate the ADC using a GUI based interface.
- [ADS8598H](https://www.ti.com/product/ADS8598H)
- [ADS8598S](https://www.ti.com/product/ADS8598S)
- [ADS8588H](https://www.ti.com/product/ADS8588H)
- [ADS8588S](https://www.ti.com/product/ADS8588S)

The example does the below
- PRU controls the ADC and reads the samples
- PRU program implements ADC samples transfer interface
- PRU then writes the samples to a shared memory and creates an interrupt after writing 1 block of data
- On interrupt R5F then reads the data and prints it on console

# Supported Combinations {#EXAMPLES_PRU_ADC_ADS85x8_DEMO_COMBOS}

\cond SOC_AM64X || SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ICSSG          | ICSSG0 - PRU0
 Toolchain      | ti-arm-clang, pru-cgt
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/pru_io/adc/ads85x8/

\endcond

# Steps to Run the Example

- Setup:  The ADC EVM is connected to AM64x/AM243x GP EVM using an Adapter Board inbetween.
    When using "T&M SEM Adapter Board", refer to \ref EXAMPLES_PRU_ADC_ADS85X8_IMPORTANT_USAGE_GUIDELINES for additional details.

    \imageStyle{AM64_adapter_adc_setup.png,width:20%}
    \image html AM64_adapter_adc_setup.png " "

- **When using CCS projects to build**, import the CCS project from the above mentioned Example folder path for system_freertos_prufw (It will automatically import R5F and PRU project) for the required combination.

- On importing the project, SysConfig will contain an ADC Config instance with default configurations which are compatible with the [ADC-PHI-PRU-EVM](https://www.ti.com/tool/ADC-PHI-PRU-EVM) Adapter Board, Use R5F SysConfig to configure the required settings for the ADC IC.

    \imageStyle{pru_adc_syscfg.png,width:60%}
    \image html pru_adc_syscfg.png "ADC SysConfig Options"
    \par
    You will need to edit linker file whenever changes are made to the configuration for \ref DRIVERS_PRU_IPC module, refer \ref DRIVERS_PRU_IPC_LINKER

- Build only the system_freertos_prufw using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
     - Build Flow: Once you click on build in system_freertos_prufw, first, PRU project is compiled according to the configurations set in PRU SysConfig. Finally the R5F project will be compiled using both the generated R5F SysConfig files and PRU project binaries or PRU Firmware header file.

    \note
    Prerequisite: [PRU-CGT-2-3](https://www.ti.com/tool/PRU-CGT) (ti-pru-cgt) should be installed at: `C:/ti/`

- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

## Features

Supported:

- Parallel 8 Bit Interface
- Parallel 16 Bit Interface
- Interrupt on receiving a block of data
- ADC, Pinmux and other configurations through SysConfig interface

Not Supported:

- Serial Interface

# Important Usage Guidelines {#EXAMPLES_PRU_ADC_ADS85X8_IMPORTANT_USAGE_GUIDELINES}

To evaluate ADS8598H use the [ADS8588SEVM-PDK](https://www.ti.com/tool/ADS8588SEVM-PDK) but solder the ADS8598H part in place of ADS8588S.
To connect the boards use T&M SEM Adapter Board, and connect it with ADC EVM and [AM64x/AM243x GP EVM](https://www.ti.com/tool/TMDS64GPEVM)

- When interfacing with T&M SEM Adapter Board (deprecated), AM64x/AM243x GP EVM requires some hardware changes as mentioned:

  \imageStyle{am64x_hardware_changes_sem_board.png,width:50%}
  \image html am64x_hardware_changes_sem_board.png "AM64x/AM243x GP EVM Changes"

    - Make sure the J9 jumper on the adapter board is in "Parallel" mode for ADS85xx adcs for all modes of interface (serial, parallel or byte-parallel).
    - From now on, T&M SEM Adapter Board won't be available for purchase, and Schematics and other details about this board can be found at: \htmllink{../am64x_am243x/T_M_SEM_Adapter_Board_files.zip, T&M SEM Adapter Board files}
    - [ADC-PHI-PRU-EVM Adapter Board](https://www.ti.com/tool/ADC-PHI-PRU-EVM) is supported from now on [orderable from May 2022]. Use of T&M SEM Adapter Board is deprecated.

# Important files and directory structure

<table>
<tr>
    <th>Folder/Files
    <th>Description
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/source/pru_io/</td></tr>
<tr>
    <td>firmware/
    <td>PRU source files providing macros for interfaces, pru_ipc, etc.</td>
</tr>
<tr>
    <td>driver/
    <td>PRU IPC source files for sending ADC samples from PRU to R5F</td>
</tr>
</table>

# Sample Output

Shown below is a sample output when the application is built and run in debug mode:

\code
--------------------------------------------------
Initializing ADC
--------------------------------------------------
Powering up ADC
--------------------------------------------------
Resetting ADC
--------------------------------------------------
Starting ADC Conversion
Started

----- Channel: 1 -----
Sample 1:  50308
Sample 2:  50304
Sample 3:  50304
Sample 4:  50304

----- Channel: 2 -----
Sample 1:  50308
Sample 2:  50304
Sample 3:  50304
Sample 4:  50304

\endcode

# Sample output with sinwave given as analog input to ADC

- [Precision signal injector](https://www.ti.com/tool/PSIEVM) is used to give analog input to ADC which can be configured using [GUI](https://www.ti.com/tool/download/SBAC170), use the configurations shown below

    \imageStyle{precision_signal_generator_gui.png,width:50%}
    \image html precision_signal_generator_gui.png " "

- [CCS graph tool](https://software-dl.ti.com/ccs/esd/documents/users_guide/ccs_debug-graphs.html) can be used to plot graph using digital data from ADC, PRU IPC start buffer address for each channel can found in linker command file of R5F project, sample output when sin wave is given as input is shown below

    \imageStyle{sinwave_output.png,width:50%}
    \image html sinwave_output.png " "

# Implementation Details

Refer here for complete implementation explanation: \subpage EXAMPLES_PRU_ADC_ADS85X8_DETAILS