#  CCS Launch, Load and Run {#CCS_LAUNCH_PAGE}

[TOC]

\if SOC_AM65X
\note The steps on this page should be done each time IDK is power cycled or when a
      new CCS session is started.
\else
\note The steps on this page should be done each time EVM is power cycled or when a
      new CCS session is started.

\endif
## Prerequisites {#PREREQUISITES}

A quick recap of the steps that need to have been done before you proceed
- Make sure you have installed CCS as mentioned in \ref CCS_SETUP_PAGE
- Make sure the UART port used for console is identified as mentioned in \ref CCS_UART_TERMINAL
\if SOC_AM65X
- Make sure you have the IDK power cable, JTAG cable, UART cable connected as shown in \ref EVM_CABLES
\else
- Make sure you have the EVM power cable, JTAG cable, UART cable connected as shown in \ref EVM_CABLES
\endif
\cond SOC_AM273X
- Make sure you have done the steps for a SOC initialization method using \ref EVM_SOC_INIT
\endcond
\cond SOC_AM263X
- Make sure to follow below steps while running multi core applications.
  - Edit the CCS gel file at "{CCS_Installation_directory}\ccs1240\ccs\ccs_base\emulation\gel\AM263x\am263x.gel"
      \imageStyle{am263x_dualcore_gel_edit_path.PNG,width:50%}
      \image html am263x_dualcore_gel_edit_path.PNG "Gel file path"

  - Modify Line 113 from "mode = AM263x_Check_supported_mode();" to "mode = 1;"
      \imageStyle{am263x_dualcore_gel_update.png,width:50%}
      \image html am263x_dualcore_gel_update.png "Gel file update"

  - With this change, CCS always configures the device in "Dual Core Mode" and all four R5 cores can be used.
  - To revert to Lockstep Mode, undo this change.
\endcond
\cond SOC_AWR294X || SOC_AM263X || SOC_AM263PX || SOC_AM243X || SOC_AM64X || SOC_AM261X
- Make sure you have done the steps for a SOC initialization method
  - **RECOMMENDED** method is \ref EVM_FLASH_SOC_INIT
  - Other options, if recommended method cannot be used, are mentioned in \ref EVM_SOC_INIT
- Make sure EVM boot mode switch is setup correctly based on the SOC initilization method
\endcond
\cond SOC_AM65X
- Make sure you have done the steps for a SOC initialization method
  - Recommended method is \ref EVM_SOC_INIT
- Make sure EVM boot mode switch is setup correctly based on the SOC initilization method
  - For the **RECOMMENDED** method, \ref EVM_SOC_INIT, the boot mode should be \ref BOOTMODE_NOBOOT
\endcond
\cond SOC_AWR294X || SOC_AM263X || SOC_AM263PX || SOC_AM261X
  - For the **RECOMMENDED** method, \ref EVM_FLASH_SOC_INIT, the boot mode should be \ref BOOTMODE_QSPI
\endcond
\cond SOC_AM243X || SOC_AM64X
  - For the **RECOMMENDED** method, \ref EVM_FLASH_SOC_INIT, the boot mode should be \ref BOOTMODE_OSPI
\endcond
\cond SOC_AM62X
- Make sure you have done the steps for a SOC initialization method as per \ref EVM_SOC_INIT
\endcond
\if SOC_AM65X
- Make sure the UART or CCS console logs on doing **IDK POWER-ON** indicate that SOC initization is successful
\else
- Make sure the UART or CCS console logs on doing **EVM POWER-ON** indicate that SOC initization is successful
\endif
- Make sure you have built the example of interest as mentioned in \ref GETTING_STARTED_BUILD

## Launch CCS {#CCS_LAUNCH}

- Launch the target configuration created with \ref CCS_NEW_TARGET_CONFIG

    \imageStyle{ccs_launch_00.png,width:40%}
    \image html ccs_launch_00.png "Launch Target Configuration"

- You will see the @VAR_SOC_NAME target configuration in the "Debug" window as shown below

    \imageStyle{ccs_launch_01.png,width:40%}
    \image html ccs_launch_01.png "Target Configuration After Launch"

\cond SOC_AM273X
- When using the \ref EVM_FLASH_SOC_INIT you need to remove the gel files from CS_DAP_0 core.
  You can skip this step when using the \ref EVM_SOC_INIT_NOBOOT_MODE

  -  Right click on the launhched target config and click show all cores
      \imageStyle{ccs_launch_02.png,width:40%}
      \image html ccs_launch_02.png "Show All Cores"

  -  Right click on the CS_DAP_0 core and open the gel file view
      \imageStyle{ccs_launch_04.png,width:40%}
      \image html ccs_launch_04.png "Open gel files view"

  -  Remove the gel files
      \imageStyle{ccs_launch_05.png,width:40%}
      \image html ccs_launch_05.png "Remove gel files"

\endcond

## Load and run example binaries {#CCS_LOAD_RUN}

\cond SOC_AM64X
\note When debugging R5/M4 Core applications along with Linux running on A53 core,
user can wait for Linux to boot till prompt and connect to the core of interest and
continue debugging or reload application of interest on the cores.
\endcond

\if SOC_AM65X
- **POWER-ON** the IDK
\else
- **POWER-ON** the EVM
\endif

\cond SOC_AWR294X || SOC_AM263X || SOC_AM263PX || SOC_AM243X || SOC_AM64X || SOC_AM261X
- If you dont see the expected SOC initialization logs on UART or CCS console, then recheck your \ref EVM_SETUP_PAGE
\endcond

\cond SOC_AM62X
- Wait for the Linux to be up on the A53 core.
    \imageStyle{linux_boot_01.png,width:60%}
    \image html linux_boot_01.png "Linux terminal"
\endcond


\cond !SOC_AM62X
- Connect the target CPU of interest if not already connected. For the "hello world" example this is `MAIN_Cortex_R5_0_0`

    \imageStyle{ccs_load_run_00.png,width:50%}
    \image html ccs_load_run_00.png "Connect CPU"

\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
- This should initialize the device and initialize R5 to be able to connect
    \imageStyle{ccs_gel_output_1.PNG,width:50%}
    \image html ccs_gel_output_1.PNG " "
    \imageStyle{ccs_gel_output_2.PNG,width:50%}
    \image html ccs_gel_output_2.PNG "GEL Output"
\endcond


\cond SOC_AM62X
- Connect the target CPU of interest if not already connected. For the "hello world" example this is `BLAZAR_Cortex_M4F_1`

    \imageStyle{ccs_load_run_00.png,width:60%}
    \image html ccs_load_run_00.png "Connect CPU"

\endcond



\cond SOC_AM273X
- when using the \ref EVM_SOC_INIT_NOBOOT_MODE connecting to the R5 core will run the gel files and you should be able to see the gel logs as below

    \imageStyle{ccs_load_run_09.png,width:50%}
    \image html ccs_load_run_09.png "Gel Output"

\endcond
- Reset the CPU

    \imageStyle{ccs_load_run_01.png,width:50%}
    \image html ccs_load_run_01.png "Reset CPU"

- Load program on the CPU

    \imageStyle{ccs_load_run_02.png,width:50%}
    \image html ccs_load_run_02.png "Load Program"

- **When using makefiles to build**,
  - In the "Load Program" dialog, select "Browse" and select the program from `examples/{example folder}/{board}/{cpu}_{os}/{compiler}` as shown below for the "hello world" program.

    \imageStyle{ccs_load_run_03.png,width:40%}
    \image html ccs_load_run_03.png "Select Program for Makefile Build"

- **When using CCS projects**,
  - In the "Load Program" dialog, select "Browse Project", select the project and then select the program as shown below,

    \imageStyle{ccs_load_run_04.png,width:40%}
    \image html ccs_load_run_04.png "Select Program for CCS Projects Build"

    \imageStyle{ccs_load_run_05.png,width:25%}
    \image html ccs_load_run_05.png "Select the Program from CCS Project"

- After the program is loaded, you will see the program is halted at "main" as shown below

    \imageStyle{ccs_load_run_06.png,width:40%}
    \image html ccs_load_run_06.png "Program at main()"

- Depending on the example you are running, you may need to load more programs on other CPUs
  for the example to work as expected. Typically, this is needed for multi-core interprocessor
  communication (IPC) examples. For the "hello world" program shown above, no more programs need to be loaded.

- Select "Resume" to run the program and debug using the various CCS debug options

    \imageStyle{ccs_load_run_07.png,width:30%}
    \image html ccs_load_run_07.png "Run the Program"

- The program output will be seen on CCS console, and/or UART terminal, if enabled.
  Below shows a sample output on both CCS console and UART console, after running the "hello world" program.

    \imageStyle{ccs_load_run_08.png,width:50%}
    \image html ccs_load_run_08.png "Run the Program"

\cond SOC_AM273X

\note When loading the examples on R5 core using the XDS560 or XDS200 CCS Hangs. Use below workaround

 - After connecting to the R5 Core

 - Open the Expression Window and enter as below

        symbol_loader=1

 - Now load the example as shown above

 - This is a known issue in CCS. Refer the below link for details https://sir.ext.ti.com/jira/browse/EXT_EP-10638

\endcond
## Re-load and run example binaries

- Here simply repeat the steps shows in \ref CCS_LOAD_RUN

- It is especially important to "reset the CPU" before reloading the program.

\if SOC_AM65X
- In most cases, you dont need to power-cycle the IDK to reload the program or load a new program.
\else
- In most cases, you dont need to power-cycle the EVM to reload the program or load a new program.

\endif
- In some cases, depending on whether the previous program execution was successful or not, the
  CPU or some SOC peripheral may be in a exception or hang state.
  In this case program reload may not work.
\if SOC_AM65X
  - **SOLUTION**: Power cycle the IDK and repeat all steps shown on this page.
\else
  - **SOLUTION**: Power cycle the EVM and repeat all steps shown on this page.
\endif
