#  IDK Setup {#IDK_SETUP_PAGE}

[TOC]

\cond SOC_NONE
\note Refer to EVM page for more details on the EVM, https://www.ti.com/tool/TMDS243GPEVM
\endcond

\attention This document contains the details for @VAR_BOARD_NAME board setup under each section.
            User should select the right board setup in the sections.

## Cable Connections {#EVM_CABLES}
### AM65X-IDK
- The figure below shows some important cable connections, ports and switches.
  - Take note of the location of the "BOOTMODE" switch, this is used to
    switch between different boot modes like OSPI, UART, SD, NOBOOT mode

  \imageStyle{evm_overview.png,width:75%}
  \image html evm_overview.png "@VAR_BOARD_NAME"

## Setup UART Terminal {#CCS_UART_TERMINAL}
### AM65X-IDK
- Many examples use a standard UART terminal to log the output from the examples.
  You can use any UART terminal program for the same. Below steps show how to setup
  the UART terminal from CCS.

- First identify the UART port as enumerated on the host machine.

  - Make sure that the IDK and UART cable connected as shown in \ref EVM_CABLES

  - In windows, you can use the "Device Manager" to see the detected UART ports
    - Search "Device Manager" in Windows Search Box in the Windows taskbar.

  - If dont see any USB serial ports listed in "Device Manager" under "Ports (COM & LPT)",
    then make sure you have installed the UART to USB driver from FTDI, https://www.ftdichip.com/FTDrivers.htm.

    \imageStyle{ccs_uart_identify.png,width:30%}
    \image html ccs_uart_identify.png "Identify UART Port in Windows Device Manager"

- In CCS, goto "View > Terminal"

    \imageStyle{ccs_uart_00.png,width:20%}
    \image html ccs_uart_00.png "UART Terminal Menu"

- Open a new UART terminal

    \imageStyle{ccs_uart_01.png,width:50%}
    \image html ccs_uart_01.png "Open New UART Terminal"

- Select the UART port, keep other options to default, i.e 115200 baud rate - 8 data bits - No parity - 1 stop bit,

  - We use the 1st USB serial port, as seen in the device manager, for below in the SDK
    - Flashing application via UART
    - Booting application via UART
    - Console output for examples which run from R5F

  - We use the 2nd USB serial port, as seen in the device manager, as terminal output for examples which run from M3F

      \imageStyle{ccs_uart_02.png,width:25%}
      \image html ccs_uart_02.png "Connect to UART port"

  - In this screenshot this happens to be COM13 and COM14 however on your machine this could be different.
    One tip to make sure there is no mistake in identifying the UART port is to disconnect all other UART to USB devices other than this IDK before checking in device manager.

## Additional Details

\note This section has more details on @VAR_BOARD_NAME. This is mainly for reference and can be skiped unless referred to by
other pages in this user guide.

### SOC Initialization Using CCS Scripting {#EVM_SOC_INIT_NOBOOT_MODE}

\note We have mentioned the steps for GP device.

#### Set Environment Variable

\note This step needs to be done once and is needed for the
  SOC initialization script `load_dmsc.js`/`load_dmsc_hsfs.js` to find certain initialization files within the SDK folder.
  This variable is not used otherwise in the build process. If you dont like adding variables in the environment, then
  you need to edit the file `${SDK_INSTALL_PATH}/tools/ccs_load/am65x/load_dmsc.js`/`${SDK_INSTALL_PATH}/tools/ccs_load/am65x/load_dmsc_hsfs.js`
  and specify the SDK path in the file itself.

- Add path to the SDK folder as a environment variable in the host machine.

- In windows, goto "Windows Task Bar Search" and search for "environment variables for your account"
        \imageStyle{ccs_setup_03.png,width:15%}
        \image html ccs_setup_03.png "Environment Variables For Your Account"


- Add a new variable named `MCU_PLUS_SDK_AM65X_PATH` and point it to the path where the SDK is installed

\imageStyle{ccs_setup_04_am65x.png,width:50%}
\image html ccs_setup_04_am65x.png "Add New Environment Variable For Your Account"

- In Linux, usually you need to add a line as below in the ${HOME}/.bashrc,

        export MCU_PLUS_SDK_AM65X_PATH=${HOME}/ti/mcu_plus_sdk_am65x_{sdk version}/



- If CCS is open, close and reopen CCS for the CCS to be able to see the updated environment variable

##### Run the SOC Initialization Script {#EVM_SOC_INIT}
###### AM65X-IDK
\attention This step needs to be done **every time** the IDK is power-cycled.

- **POWER-OFF** the IDK

- Make sure below cables are connected as shown in \ref EVM_CABLES
  - Power cable
  - JTAG cable

- Set IDK in NOBOOT mode as shown below
  \imageStyle{boot_pins_noboot_mode.png,width:30%}
  \image html boot_pins_noboot_mode.png "NO BOOT MODE"

- **POWER-ON** the IDK

- Launch the target configuration created with \ref CCS_NEW_TARGET_CONFIG

    \imageStyle{ccs_launch_00.png,width:40%}
    \image html ccs_launch_00.png "Launch Target Configuration"

- You will see the @VAR_SOC_NAME target configuration in the "Debug" window as shown below

    \imageStyle{ccs_launch_01.png,width:40%}
    \image html ccs_launch_01.png "Target Configuration After Launch"

- Goto "CCS Toolbar > View > Scripting Console"

- Type the below command in the scripting console and press "enter", to load DMSC FW and initialize the SOC
  - In Windows, assuming the SDK is installed at `C:/ti/mcu_plus_sdk_{soc}_{sdk version}`

        loadJSFile "C:/ti/mcu_plus_sdk_{soc}_{sdk version}/tools/ccs_load/am65x/load_dmsc.js" (GP device)

    \imageStyle{ccs_load_dmsc_00.png,width:50%}
    \image html ccs_load_dmsc_00.png "Scripting Console"

- In Linux, run the same command, only the path would be a Linux path like `/home/{username}/ti/mcu_plus_sdk_{soc}_{sdk version}/tools/ccs_load/am65x/load_dmsc.js`
- After successful execution of this script one would see logs as below

  - In the scripting console, this is log from the script itself.
    \imageStyle{ccs_load_dmsc_01.png,width:50%}
    \image html ccs_load_dmsc_01.png "Scripting Console Log"

  - In the @VAR_SOC_NAME "CIO" console, this is log from the R5F.
    \imageStyle{ccs_load_dmsc_02.png,width:50%}
    \image html ccs_load_dmsc_02.png "@VAR_SOC_NAME CIO Console R5F Log"

  - In the @VAR_SOC_NAME console, this is log from the GEL scripts.
    \imageStyle{ccs_load_dmsc_03.png,width:50%}
    \image html ccs_load_dmsc_03.png "Select @VAR_SOC_NAME GEL Console"

    \imageStyle{ccs_load_dmsc_04.png,width:50%}
    \image html ccs_load_dmsc_04.png "@VAR_SOC_NAME Console GEL Log"

- For success, all the three consoles should have no errors in their logs.

- If the script is run without providing power to the IDK or if the IDK BOOTMODE is
  not set to \ref BOOTMODE_NOBOOT then you will see errors in the console and/or unexpected behaviour and error messages.
  - **SOLUTION**: Power cycle IDK and repeat the steps.

### BOOT MODE

#### NOBOOT MODE  {#BOOTMODE_NOBOOT}
##### AM65X-IDK
This mode is used in conjunction with the `load_dmsc.js` script described here \ref EVM_SOC_INIT_NOBOOT_MODE,
    \code
    BOOTMODE [ 0 :  7 ] (SW2) = 0010 0000
    BOOTMODE [ 8 : 15 ] (SW3) = 0000 0000
    BOOTMODE [0:1]      (SW4) = 00
    \endcode

  \imageStyle{boot_pins_noboot_mode.png,width:30%}
  \image html boot_pins_noboot_mode.png "NO BOOT MODE"


## Troubleshooting IDK issues {#TROUBLESHOOT_ISSUES}

 - JTAG connection fails on some IDKs with the following error. Need to connect the JTAG cable after board is powered on.

  \imageStyle{jtag_connection_error.PNG,width:30%}
  \image html jtag_connection_error.PNG "JTAG Connection Error Dialog"
