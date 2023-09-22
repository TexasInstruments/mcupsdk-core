#  Additional Details {#ADDITIONAL_DETAILS_PAGE}

[TOC]

### Alternate Method to Build and Run the examples

\note These steps are optional.

Below method cab be used for running examples built with makefiles

 - **Step 1:** Build a "hello world" example for the EVM, see \subpage GETTING_STARTED_BUILD

 - **Step 2:** Load and run the "hello world" example on the EVM, see \subpage CCS_LAUNCH_PAGE

 - **Step 3:** Flash the "hello world" example on the EVM and boot without CCS, see \subpage GETTING_STARTED_FLASH

#### SOC Initialization {#EVM_SOC_INIT}

Before any program can be loaded and run on the EVM, the SOC needs to be initialized.
Below sections describes the various options available for SOC initialization.

#### SOC Initialization Using Gel {#EVM_SOC_INIT_NOBOOT_MODE}

###### AM263PX-CC
\attention This step needs to be done **every time** the AM263PX-CC is power-cycled.

- **POWER-OFF** the AM263PX-CC

- Make sure below cables are connected as shown in \ref EVM_CABLES
  - Power cable
  - JTAG cable

- Set AM263PX-CC in NOBOOT mode as shown below

  \imageStyle{boot_pins_noboot_mode.PNG,width:30%}
  \image html boot_pins_noboot_mode.PNG "NO BOOT MODE"

- **POWER-ON** the AM263PX-CC

- Launch the target configuration created with \ref CCS_NEW_TARGET_CONFIG

    \imageStyle{ccs_launch_00.png,width:40%}
    \image html ccs_launch_00.png "Launch Target Configuration"

- You will see the @VAR_SOC_NAME target configuration in the "Debug" window as shown below

    \imageStyle{ccs_launch_02.png,width:40%}
    \image html ccs_launch_02.png "Target Configuration After Launch"

- Connect the R5_0 core and it will initialise the core

    \imageStyle{ccs_gel_output_1.PNG,width:50%}
    \image html ccs_gel_output_1.PNG " "
    \imageStyle{ccs_gel_output_2.PNG,width:50%}
    \image html ccs_gel_output_2.PNG "GEL Output"

#### SOC Initialization using the Binary Flashed in OSPI memory {#EVM_FLASH_SOC_INIT}


##### AM263PX-CC
The `sbl_null` is a secondary bootloader which doesn't load any application binary, but just does the SOC initialization and puts all the cores in WFI (Wait For Interrupt) mode.

- This is a recommended one time step that needs to be done before
           you can load and run programs via CCS

- If this step fails, maybe due to bad flash in EVM, then try one of the other SOC initialization steps
           mentioned at \ref EVM_SOC_INIT

- This step needs to be done once unless the OSPI flash has been erased
           or some other application has been flashed

- A quick recap of steps done so far that are needed for the flashing to work
  - Make sure the UART port used for terminal is identified as mentioned in \ref CCS_UART_TERMINAL
  - Make sure python3 is installed as mentioned in \ref INSTALL_PYTHON3
  - Make sure you have the EVM power cable and UART cable connected as shown in \ref EVM_CABLES

- **POWER-OFF** the EVM

- Set boot mode to UART BOOTMODE as shown in below image

  \imageStyle{boot_pins_uart_mode.PNG,width:30%}
  \image html boot_pins_uart_mode.PNG "UART BOOT MODE"

- **POWER-ON** the EVM

- You should see character "C" getting printed on the UART terminal every 2-3 seconds as shown below

  \imageStyle{uart_rom_boot.png,width:80%}
  \image html uart_rom_boot.png "UART output in UART BOOT MODE"

- Close the UART terminal as shown below. This is important, else the UART script in next step wont be able to connect to the UART port.

  \imageStyle{ccs_uart_close.png,width:80%}
  \image html ccs_uart_close.png "Close UART terminal"

- Open a command prompt and run the below command to flash the SOC initialization binary to the EVM.

        cd ${SDK_INSTALL_PATH}/tools/boot
        python uart_uniflash.py -p COM<x> --cfg=sbl_prebuilt/@VAR_BOARD_NAME_LOWER/default_sbl_null.cfg

  - Here COM<x> is the port name of the identified UART port in Windows.
  - On Linux,
    - The name for UART port is typically something like `/dev/ttyUSB0`
    - On some Linux systems, one needs to use `python3` to invoke python3.x, just `python` command may invoke python 2.x which will not work with the flashing script.

- When the flashing is in progress you will see something like below

  \imageStyle{flash_soc_init_in_progress.png,width:100%}
  \image html flash_soc_init_in_progress.png "Flash in progress"

- After all the flashing is done, you will see something like below

  \imageStyle{flash_soc_init_success.png,width:80%}
  \image html flash_soc_init_success.png "Flashing successful"

- If flashing has failed, see \ref TOOLS_FLASH_ERROR_MESSAGES, and resolve the errors.

- If flashing is successful, do the next steps ...

- **POWER-OFF** the EVM

- Switch the EVM boot mode to OSPI mode as shown below,

  \imageStyle{boot_pins_qspi_mode.PNG,width:30%}
  \image html boot_pins_qspi_mode.PNG "OSPI BOOT MODE"

- Re-connect the UART terminal in CCS window as shown in \ref CCS_UART_TERMINAL

- **POWER-ON** the EVM


#### Run the example

- Now you can run the example built (see \ref CCS_LAUNCH_PAGE)

- Connect to any of the Cortex_R5_0/Cortex_R5_1/C66xx_DSP cores, load programs and run (see \ref CCS_LOAD_RUN)

