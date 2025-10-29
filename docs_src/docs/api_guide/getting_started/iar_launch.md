#  IAR Launch, Load and Run {#IAR_LAUNCH_PAGE}

[TOC]

\note The steps on this page should be done each time EVM is power cycled or when a
      new IAR session is started.

## Prerequisites {#IAR_PREREQUISITES}

A quick recap of the steps that need to have been done before you proceed
- Make sure you have installed IAR Embedded Workbench as mentioned in \ref IAR_SETUP_PAGE
- Make sure the UART port used for console is identified as mentioned in \ref CCS_UART_TERMINAL
- Make sure you have the EVM power cable, JTAG cable, UART cable connected as shown in \ref EVM_CABLES
- Make sure you have done the steps for a SOC initialization method \ref EVM_FLASH_SOC_INIT
- Make sure EVM boot mode switch is setup correctly based on the SOC initilization method
  - For the **RECOMMENDED** method, \ref EVM_FLASH_SOC_INIT, the boot mode should be \ref BOOTMODE_OSPI
- Make sure the UART logs on doing **EVM POWER-ON** indicate that SOC initization is successful
- Make sure you have built the example of interest as mentioned in \ref GETTING_STARTED_BUILD

## Load and run example binaries in IAR Embedded Workbench {#IAR_LOAD_RUN}

- Launch IAR Embedded Workbench IDE
- **POWER-ON** the EVM
- Make sure the `SBL NULL` is flashed on the board, and ensure the UART logs of SBL NULL get printed. Refer \ref IAR_SBLL_NULL_LOAD
- **When using IAR EW for build**
    - Open the workspace and project need to be flashed.
    - Build the project.
    - Navigate to Toolbar > `Project` > `Download and Debug`
    
    \imageStyle{iar_project_download_menu.png,width:15%}
    \image html iar_project_download_menu.png "Download and Debug Program"

    - Now the program will be halted at `main()`
    - Navigate to Toolbar > `View` > `Terminal IO` to open the terminal to view the console logs
       
    \imageStyle{iar_terminal_menu.png,width:15%}
    \image html iar_terminal_menu.png "Terminal IO"

    - Click **Go** button on the toolbar to run the program and debug using various IAR debug options.
       
    \imageStyle{iar_debug_go.png,width:25%}
    \image html iar_debug_go.png "Debug Options"

    - The program output can be seen on the UART and Terminal log.
    - Below shows the output of the "hello world" example
     
    \imageStyle{iar_hello_world_output.png,width:75%}
    \image html iar_hello_world_output.png "Hello world program output"

- **When using Makefile for build**
    - Open any existing project

    - Right click on the project and select the option **Add external binary** and browse to the .out file location.
    \imageStyle{iar_add_external_binary.png,width:20%}
    \image html iar_add_external_binary.png "Add external binary"

    - Now the binary is loaded to the project, Right click on the .out file and make sure the "Set as Debug Target" is enabled.
    \imageStyle{iar_set_target_build.png,width:20%}
    \image html iar_set_target_build.png "Set Target binary for debug"
    
    - Click on Project > Download and Debug to load the binary. Continue as the steps in the previous section.
