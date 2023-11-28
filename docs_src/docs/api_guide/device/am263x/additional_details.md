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

#### SOC Initialization Using CCS Scripting {#EVM_SOC_INIT_NOBOOT_MODE}

##### Set Environment Variable

\note This step needs to be done once and is needed for the
  SOC initialization script `load_sbl.js` to find certain initialization files within the SDK folder.
  This variable is not used otherwise in the build process. If you dont like adding variables in the environment, then
  you need to edit the file `${SDK_INSTALL_PATH}/tools/ccs_load/am263x/load_sbl.js` and specify the SDK path in the file itself.

- Add path to the SDK folder as a environment variable in the host machine.

- In windows, goto "Windows Task Bar Search" and search for "environment variables for your account"
        \imageStyle{ccs_setup_03.png,width:15%}
        \image html ccs_setup_03.png "Environment Variables For Your Account"

- Add a new variable named `MCU_PLUS_SDK_AM263X_PATH` and point it to the path where the SDK is installed

\imageStyle{ccs_setup_04.png,width:50%}
\image html ccs_setup_04.png "Add New Environment Variable For Your Account"

- In Linux, usually you need to add a line as below in the ${HOME}/.bashrc,

        export MCU_PLUS_SDK_AM243X_PATH=${HOME}/ti/mcu_plus_sdk_am263x_{sdk version}/

- If CCS is open, close and reopen CCS for the CCS to be able to see the updated environment variable

##### Run the SOC Initialization Script
###### AM263X-LP
\attention This step needs to be done **every time** the AM263X-LP is power-cycled.

- **POWER-OFF** the AM263X-LP

- Make sure below cables are connected as shown in \ref EVM_CABLES
  - Power cable
  - JTAG cable

- Set AM263X-LP in DEVBOOT mode as shown below

  \imageStyle{am263x_lp_boot_pins_noboot_mode.png,width:30%}
  \image html am263x_lp_boot_pins_noboot_mode.png "DEVBOOT MODE"

- **POWER-ON** the AM263X-LP

- Launch the target configuration created with \ref CCS_NEW_TARGET_CONFIG

    \imageStyle{ccs_launch_00.png,width:40%}
    \image html ccs_launch_00.png "Launch Target Configuration"

- You will see the @VAR_SOC_NAME target configuration in the "Debug" window as shown below

    \imageStyle{ccs_launch_02.png,width:40%}
    \image html ccs_launch_02.png "Target Configuration After Launch"

- Goto "CCS Toolbar > View > Scripting Console"

- Type the below command in the scripting console and press "enter", to load DMSC FW and initialize the SOC
  - In Windows, assuming the SDK is installed at `C:/ti/mcu_plus_sdk_{soc}_{sdk version}`

        loadJSFile "C:/ti/mcu_plus_sdk_{soc}_{sdk version}/tools/ccs_load/am263x/load_sbl.js"

    \imageStyle{ccs_load_dmsc_00.png,width:50%}
    \image html ccs_load_dmsc_00.png "Scripting Console"

- In Linux, run the same command, only the path would be a Linux path like `/home/{username}/ti/mcu_plus_sdk_{soc}_{sdk version}/tools/ccs_load/am263x/load_sbl.js`

- After successful execution of this script one would see logs as below

  - In the scripting console, this is log from the script itself.
    \imageStyle{ccs_load_dmsc_01.png,width:50%}
    \image html ccs_load_dmsc_01.png "Scripting Console Log"

- In case of success, there should be no error logs in the scripting console. The core will continue to run and the user can halt and reset the core.

- If the script is run without providing power to the AM263X-LP or if the AM263X-LP BOOTMODE is
  not set to \ref BOOTMODE_NOBOOT then you will see errors in the console and/or unexpected behaviour and error messages.
  - **SOLUTION**: Power cycle AM263X-LP and repeat the steps.

###### AM263X-CC
\attention This step needs to be done **every time** the AM263X-CC is power-cycled.

- **POWER-OFF** the AM263X-CC

- Make sure below cables are connected as shown in \ref EVM_CABLES
  - Power cable
  - JTAG cable

- Set AM263X-CC in DEVBOOT mode as shown below

  \imageStyle{boot_pins_noboot_mode.PNG,width:30%}
  \image html boot_pins_noboot_mode.PNG "DEVBOOT MODE"

- **POWER-ON** the AM263X-CC

- Launch the target configuration created with \ref CCS_NEW_TARGET_CONFIG

    \imageStyle{ccs_launch_00.png,width:40%}
    \image html ccs_launch_00.png "Launch Target Configuration"

- You will see the @VAR_SOC_NAME target configuration in the "Debug" window as shown below

    \imageStyle{ccs_launch_02.png,width:40%}
    \image html ccs_launch_02.png "Target Configuration After Launch"

- Goto "CCS Toolbar > View > Scripting Console"

- Type the below command in the scripting console and press "enter", to load DMSC FW and initialize the SOC
  - In Windows, assuming the SDK is installed at `C:/ti/mcu_plus_sdk_{soc}_{sdk version}`

        loadJSFile "C:/ti/mcu_plus_sdk_{soc}_{sdk version}/tools/ccs_load/am263x/load_sbl.js"

    \imageStyle{ccs_load_dmsc_00.png,width:50%}
    \image html ccs_load_dmsc_00.png "Scripting Console"

- In Linux, run the same command, only the path would be a Linux path like `/home/{username}/ti/mcu_plus_sdk_{soc}_{sdk version}/tools/ccs_load/am263x/load_sbl.js`

- After successful execution of this script one would see logs as below

  - In the scripting console, this is log from the script itself.
    \imageStyle{ccs_load_dmsc_01.png,width:50%}
    \image html ccs_load_dmsc_01.png "Scripting Console Log"

- In case of success, there should be no error logs in the scripting console. The core will continue to run and the user can halt and reset the core.

- If the script is run without providing power to the AM263X-CC or if the AM263X-CC BOOTMODE is
  not set to \ref BOOTMODE_NOBOOT then you will see errors in the console and/or unexpected behaviour and error messages.
  - **SOLUTION**: Power cycle AM263X-CC and repeat the steps.

#### SOC Initialization using the Binary Flashed in QSPI memory {#EVM_FLASH_SOC_INIT}


### AM263X-CC
The `sbl_null` is a secondary bootloader which doesn't load any application binary, but just does the SOC initialization and puts all the cores in WFI (Wait For Interrupt) mode.

- This is a recommended one time step that needs to be done before
           you can load and run programs via CCS

- If this step fails, maybe due to bad flash in EVM, then try one of the other SOC initialization steps
           mentioned at \ref EVM_SOC_INIT

- This step needs to be done once unless the QSPI flash has been erased
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

- Switch the EVM boot mode to QSPI mode as shown below,

  \imageStyle{boot_pins_qspi_mode.PNG,width:30%}
  \image html boot_pins_qspi_mode.PNG "QSPI BOOT MODE"

- Re-connect the UART terminal in CCS window as shown in \ref CCS_UART_TERMINAL

- **POWER-ON** the EVM

### AM263X-LP
The `sbl_null` is a secondary bootloader which doesn't load any application binary, but just does the SOC initialization and puts all the cores in WFI (Wait For Interrupt) mode.

- This is a recommended one time step that needs to be done before
           you can load and run programs via CCS

- If this step fails, maybe due to bad flash in EVM, then try one of the other SOC initialization steps
           mentioned at \ref EVM_SOC_INIT

- This step needs to be done once unless the QSPI flash has been erased
           or some other application has been flashed

- A quick recap of steps done so far that are needed for the flashing to work
  - Make sure the UART port used for terminal is identified as mentioned in \ref CCS_UART_TERMINAL
  - Make sure python3 is installed as mentioned in \ref INSTALL_PYTHON3
  - Make sure you have the EVM power cable and UART cable connected as shown in \ref EVM_CABLES

- **POWER-OFF** the EVM

- Set boot mode to UART BOOTMODE as shown in below image

  \imageStyle{am263x_lp_boot_pins_uart_mode.png,width:30%}
  \image html am263x_lp_boot_pins_uart_mode.png "UART BOOT MODE"

- **POWER-ON** the EVM

- You should see character "C" getting printed on the UART terminal every 2-3 seconds as shown below

  \imageStyle{uart_rom_boot.png,width:80%}
  \image html uart_rom_boot.png "UART output in UART BOOT MODE"

- Close the UART terminal as shown below. This is important, else the UART script in next step wont be able to connect to the UART port.

  \imageStyle{ccs_uart_close.png,width:80%}
  \image html ccs_uart_close.png "Close UART terminal"

- Open a command prompt and run the below command to flash the SOC initialization binary to the EVM.

        cd ${SDK_INSTALL_PATH}/tools/boot
        python uart_uniflash.py -p COM<x> --cfg=sbl_prebuilt/@VAR_LP_BOARD_NAME_LOWER/default_sbl_null.cfg

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

- Switch the EVM boot mode to QSPI mode as shown below,

  \imageStyle{am263x_lp_boot_pins_qspi_mode.png,width:30%}
  \image html am263x_lp_boot_pins_qspi_mode.png "QSPI BOOT MODE"

- Re-connect the UART terminal in CCS window as shown in \ref CCS_UART_TERMINAL

- **POWER-ON** the EVM



#### Run the example

- Now you can run the example built (see \ref CCS_LAUNCH_PAGE)

- Connect to any of the Cortex_R5_0/Cortex_R5_1 cores, load programs and run (see \ref CCS_LOAD_RUN)

