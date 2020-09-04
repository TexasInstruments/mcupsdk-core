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

##### SOC Initialization Using CCS {#EVM_SOC_INIT_NOBOOT_MODE}

- **POWER-OFF** the EVM

- Make sure below cables are connected as shown in \ref EVM_CABLES
  - Power cable
  - JTAG cable

- the boot mode should be \ref BOOTMODE_NOBOOT

- **POWER-ON** the EVM

- Now you can build a example of interest (see \ref GETTING_STARTED_BUILD) and then run it (see \ref CCS_LAUNCH_PAGE)

- Now you can Connect to any of the Cortex_R5_0/Cortex_R5_1/C66xx_DSP cores, load programs and run (see \ref CCS_LOAD_RUN)

##### SOC Initialization using the Binary Flashed in QSPI memory {#EVM_FLASH_SOC_INIT}

\note This is a one time step that needs to be done before
           you can load and run programs via CCS

\note If this step fails, maybe due to bad flash in EVM, then try one of the other SOC initialization steps
           mentioned at \ref EVM_SOC_INIT

\note This step needs to be done once unless the QSPI flash has been erased
           or some other application has been flashed

- A quick recap of steps done so far that are needed for the flashing to work
  - Make sure the UART port used for terminal is identified as mentioned in \ref CCS_UART_TERMINAL
  - Make sure python3 is installed as mentioned in \ref INSTALL_PYTHON3
  - Make sure you have the EVM power cable and UART cable connected as shown in \ref EVM_CABLES

- **POWER-OFF** the EVM

- The boot mode should be \ref BOOTMODE_UART

- **POWER-ON** the EVM

- You should see character "C" getting printed on the UART terminal every 1-2 seconds as shown below

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

  \imageStyle{flash_soc_init_in_progress.png,width:80%}
  \image html flash_soc_init_in_progress.png "Flash in progress"

- After all the flashing is done, you will see something like below

  \imageStyle{flash_soc_init_success.png,width:80%}
  \image html flash_soc_init_success.png "Flashing successful"

- If flashing has failed, see \ref TOOLS_FLASH_ERROR_MESSAGES, and resolve the errors.

- If flashing is successful, do the next steps ...

- **POWER-OFF** the EVM

- Change the boot mode to \ref BOOTMODE_QSPI

- **POWER-ON** the EVM

- Re-connect the UART terminal in CCS window as shown in \ref CCS_UART_TERMINAL

- Press the reset button on the board as shown in \ref EVM_CABLES

- You should see output like below on the UART terminal

        Starting NULL Bootloader ...
        NULL Bootloader Execution Complete...

- Now the EVM is setup for loading and running from CCS !!!
- You dont need to do these steps again unless you have flashed some other binary to the flash.

#### Run the example

- Now you can run the example built (see \ref CCS_LAUNCH_PAGE)

- Connect to any of the Cortex_R5_0/Cortex_R5_1/C66xx_DSP cores, load programs and run (see \ref CCS_LOAD_RUN)

