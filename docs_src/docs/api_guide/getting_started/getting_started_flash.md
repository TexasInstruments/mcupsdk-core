#  Flash a Hello World example {#GETTING_STARTED_FLASH}

[TOC]

\attention Flashing a application will overwrite the SOC init application that was flashed earlier.
           So if you want to load and run from CCS again, you will need to do the SOC init steps again.
           See \ref EVM_SOC_INIT for more details.

\note See also \ref TOOLS_FLASH for more details on the flashing tools.

## Introduction

In this step we will flash the application that we have build and run using CCS to the EVM flash.
We can then boot this application without being connected to CCS via JTAG.

## Getting ready to flash the application

- A quick recap of steps done so far that are needed for the flashing to work
  - Make sure the UART port used for terminal is identified as mentioned in \ref CCS_UART_TERMINAL
  - Make sure python3 is installed as mentioned in \ref INSTALL_PYTHON3
  - Make sure you have the EVM power cable and UART cable connected as shown in \ref EVM_CABLES

- Build the hello world application as mentioned in \ref GETTING_STARTED_BUILD

- As part of the build process in the final step a file with extension `.appimage` is generated. This is the file
  we need to flash.

  - When building with makefiles and single-core projects, this file can be found here (shown for hello world example),

        ${SDK_INSTALL_PATH}/examples/hello_world/{board}/r5fss0-0_freertos/ti-arm-clang/hello_world.release.appimage

  - When building with CCS and single-core projects, this file can be found here (shown for hello world example),

        ${CCS_WORKSPACE_PATH}/hello_world_{board}_r5fss0-0_freertos_ti-arm-clang/Release/hello_world_{board}_r5fss0-0_freertos_ti-arm-clang.appimage

  - When building with makefiles and multi-core system projects, this file can be found here (shown for IPC Notify example),

        ${SDK_INSTALL_PATH}/examples/drivers/ipc/ipc_notify_echo/{board}/system_freertos_nortos/ipc_notify_echo_system.release.appimage

  - When building with CCS and multi-core system projects, this file can be found here (shown for IPC Notify example),

        ${CCS_WORKSPACE_PATH}/ipc_notify_echo_{board}_system_freertos_nortos/Release/ipc_notify_echo_system.appimage

  - **NOTE**: The folder name and file name in path can have "release", "Release" or "debug", "Debug" based on the profile that the application is built with.

- Next, we need to list the files to flash in a flash configuration file. A default configuration file can be found at below path.
  You can edit this file directly or take a copy and edit this file.

\cond SOC_AM273X || SOC_AWR294X || SOC_AM263X
        ${SDK_INSTALL_PATH}/tools/boot/sbl_prebuilt/{board}/default_sbl_qspi.cfg
\endcond
\cond SOC_AM64X || SOC_AM243X
        ${SDK_INSTALL_PATH}/tools/boot/sbl_prebuilt/{board}/default_sbl_ospi.cfg
\endcond

- Edit below line in the config file to point to your application `.appimage` file.
  Give the absolute path to the `.appimage` file or path relative to `${SDK_INSTALL_PATH}/tools/boot`. **Make sure to use forward slash `/` in the filename path**.

        --file=../../examples/drivers/ipc/ipc_notify_echo/{board}/system_freertos_nortos/ipc_notify_echo_system.release.appimage --operation=flash --flash-offset=0x80000

- This file will additionally also list the flashing application that is run on the EVM and a OSPI flash bootloader that also
  needs to be flashed. You can keep this unchanged if you have not modified these applications.

- Save and close the config file.

\cond SOC_AM64X
## Flashing the application
### AM64X-EVM
- **POWER-OFF** the EVM

- Set boot mode to UART BOOTMODE as shown in below image

  \imageStyle{boot_pins_uart_mode.png,width:30%}
  \image html boot_pins_uart_mode.png "UART BOOT MODE"

- **POWER-ON** the EVM

- You should see character "C" getting printed on the UART terminal every 2-3 seconds as shown below

  \imageStyle{uart_rom_boot.png,width:80%}
  \image html uart_rom_boot.png "UART output in UART BOOT MODE"

- Close the UART terminal as shown below. This is important, else the UART script in next step wont be able to connect to the UART port.

  \imageStyle{ccs_uart_close.png,width:80%}
  \image html ccs_uart_close.png "Close UART terminal"

- Open a command prompt and run the below command to flash the SOC initialization binary to the EVM.

        cd ${SDK_INSTALL_PATH}/tools/boot
        python uart_uniflash.py -p COM13 --cfg=sbl_prebuilt/@VAR_BOARD_NAME_LOWER/default_sbl_ospi.cfg

  - Here COM13 is the port name of the identified UART port in Windows.
  - On Linux,
    - The name for UART port is typically something like `/dev/ttyUSB0`
    - On some Linux systems, one needs to use `python3` to invoke python3.x, just `python` command may invoke python 2.x which will not work with the flashing script.

- When the flashing is in progress you will see something like below

  \imageStyle{flash_soc_init_in_progress.png,width:100%}
  \image html flash_soc_init_in_progress.png "Flash in progress"

- After all the flashing is done, you will see something like below

        C:/ti/mcu_plus_sdk/tools/boot>python uart_uniflash.py -p COM13 --cfg=sbl_prebuilt/@VAR_BOARD_NAME_LOWER/default_sbl_ospi.cfg

        Parsing config file ...
        Parsing config file ... SUCCESS. Found 3 command(s) !!!

        Executing command 1 of 3 ...
        Found flash writer ... sending sbl_prebuilt/@VAR_BOARD_NAME_LOWER/sbl_uart_uniflash.release.tiimage
        Sent flashwriter sbl_prebuilt/@VAR_BOARD_NAME_LOWER/sbl_uart_uniflash.release.tiimage of size 289169 bytes in 28.52s.

        Executing command 2 of 3 ...
        Command arguments : --file=sbl_prebuilt/@VAR_BOARD_NAME_LOWER/sbl_ospi.release.tiimage --operation=flash --flash-offset=0x0
        Sent sbl_prebuilt/@VAR_BOARD_NAME_LOWER/sbl_ospi.release.tiimage of size 241937 bytes in 25.31s.
        [STATUS] SUCCESS !!!

        Executing command 3 of 3 ...
        Command arguments : --file=C:/Users/XYZ/workspace_v10/hello_world_{board}_r5fss0-0_freertos_ti-arm-clang/Release/hello_world_{board}_r5fss0-0_freertos_ti-arm-clang.appimage --operation=flash --flash-offset=0x80000
        Sent C:/Users/XYZ/workspace_v10/hello_world_{board}_r5fss0-0_freertos_ti-arm-clang/Release/hello_world_{board}_r5fss0-0_freertos_ti-arm-clang.appimage of size 58772 bytes in 8.43s.
        [STATUS] SUCCESS !!!

        All commands from config file are executed !!!

- If flashing has failed, see \ref TOOLS_FLASH_ERROR_MESSAGES, and resolve the errors.

- If flashing is successful, do the next steps ...

### AM64X-SK
- **POWER-OFF** the AM64X-SK

- Set boot mode to UART BOOTMODE as shown in below image

  \imageStyle{sk_boot_pins_uart_mode.png,width:30%}
  \image html sk_boot_pins_uart_mode.png "UART BOOT MODE"

- **POWER-ON** the AM64X-SK

- You should see character "C" getting printed on the UART terminal every 2-3 seconds as shown below

  \imageStyle{uart_rom_boot.png,width:80%}
  \image html uart_rom_boot.png "UART output in UART BOOT MODE"

- Close the UART terminal as shown below. This is important, else the UART script in next step wont be able to connect to the UART port.

  \imageStyle{ccs_uart_close.png,width:80%}
  \image html ccs_uart_close.png "Close UART terminal"

- Open a command prompt and run the below command to flash the SOC initialization binary to the AM64X-SK.

        cd ${SDK_INSTALL_PATH}/tools/boot
        python uart_uniflash.py -p COM6 --cfg=sbl_prebuilt/@VAR_SK_BOARD_NAME_LOWER/default_sbl_ospi.cfg

  - Here COM6 is the port name of the identified UART port in Windows.
  - On Linux,
    - The name for UART port is typically something like `/dev/ttyUSB0`
    - On some Linux systems, one needs to use `python3` to invoke python3.x, just `python` command may invoke python 2.x which will not work with the flashing script.

- When the flashing is in progress you will see something like below

  \imageStyle{flash_soc_init_in_progress.png,width:100%}
  \image html flash_soc_init_in_progress.png "Flash in progress"

- After all the flashing is done, you will see something like below

        C:/ti/mcu_plus_sdk/tools/boot>python uart_uniflash.py -p COM6 --cfg=sbl_prebuilt/@VAR_SK_BOARD_NAME_LOWER/default_sbl_ospi.cfg

        Parsing config file ...
        Parsing config file ... SUCCESS. Found 3 command(s) !!!

        Executing command 1 of 3 ...
        Found flash writer ... sending sbl_prebuilt/@VAR_SK_BOARD_NAME_LOWER/sbl_uart_uniflash.release.tiimage
        Sent flashwriter sbl_prebuilt/@VAR_SK_BOARD_NAME_LOWER/sbl_uart_uniflash.release.tiimage of size 289169 bytes in 28.52s.

        Executing command 2 of 3 ...
        Command arguments : --file=sbl_prebuilt/@VAR_SK_BOARD_NAME_LOWER/sbl_ospi.release.tiimage --operation=flash --flash-offset=0x0
        Sent sbl_prebuilt/@VAR_SK_BOARD_NAME_LOWER/sbl_ospi.release.tiimage of size 241937 bytes in 25.31s.
        [STATUS] SUCCESS !!!

        Executing command 3 of 3 ...
        Command arguments : --file=C:/Users/XYZ/workspace_v10/hello_world_{board}_r5fss0-0_freertos_ti-arm-clang/Release/hello_world_{board}_r5fss0-0_freertos_ti-arm-clang.appimage --operation=flash --flash-offset=0x80000
        Sent C:/Users/XYZ/workspace_v10/hello_world_{board}_r5fss0-0_freertos_ti-arm-clang/Release/hello_world_{board}_r5fss0-0_freertos_ti-arm-clang.appimage of size 58772 bytes in 8.43s.
        [STATUS] SUCCESS !!!

        All commands from config file are executed !!!

- If flashing has failed, see \ref TOOLS_FLASH_ERROR_MESSAGES, and resolve the errors.

- If flashing is successful, do the next steps ...

## Running the flashed application
### AM64X-EVM
- **POWER-OFF** the EVM

- Switch the EVM boot mode to OSPI mode as shown below,

  \imageStyle{boot_pins_ospi_mode.png,width:30%}
  \image html boot_pins_ospi_mode.png "OSPI BOOT MODE"

- Re-connect the UART terminal in CCS window as shown in \ref CCS_UART_TERMINAL

- **POWER-ON** the EVM

- You should see the application output in UART terminal as below

        Starting OSPI Bootloader ...

        DMSC Firmware Version 21.1.1--v2021.01a (Terrific Lla
        DMSC Firmware revision 0x15
        DMSC ABI revision 3.1

        INFO: Bootloader_runCpu:151: CPU m4f0-0 is initialized to 400000000 Hz !!!
        INFO: Bootloader_runCpu:151: CPU r5f1-0  is initialized to 800000000 Hz !!!
        INFO: Bootloader_runCpu:151: CPU r5f1-1 is initialized to 800000000 Hz !!!
        INFO: Bootloader_loadSelfCpu:214: CPU r5f0-0 is initialized to 800000000 Hz !!!
        INFO: Bootloader_loadSelfCpu:214: CPU r5f0-1 is initialized to 800000000 Hz !!!
        INFO: Bootloader_loadSelfCpu:235: All done, reseting self ...

        [DPL] Hwi post ...
        [DPL] Hwi post ... DONE !!!
        [DPL] Sleep for 100 msecs ...
        [DPL] Sleep ... DONE (Measured time = 100002 usecs, CPU cycles = 438501 ) !!!
        [DPL] Running cache operations ...
        [DPL] Running cache operations ... DONE !!!
        [DPL] Heap free size = 1984 bytes
        [DPL] Allocated 1023 bytes @ 0x70086080, heap free size = 896 bytes
        [DPL] Free'ed 1023 bytes @ 0x70086080, heap free size = 1984 bytes
        All tests have passed!!

- Congratulations now the EVM is flashed with your application and you dont need CCS anymore to run the application.

### AM64X-SK
- **POWER-OFF** the AM64X-SK

- Switch the EVM boot mode to OSPI mode as shown below,

  \imageStyle{sk_boot_pins_ospi_mode.png,width:30%}
  \image html sk_boot_pins_ospi_mode.png "OSPI BOOT MODE"

- Re-connect the UART terminal in CCS window as shown in \ref CCS_UART_TERMINAL

- **POWER-ON** the AM64X-SK

- You should see the application output in UART terminal as below

        Starting OSPI Bootloader ...

        DMSC Firmware Version 21.1.1--v2021.01a (Terrific Lla
        DMSC Firmware revision 0x15
        DMSC ABI revision 3.1

        INFO: Bootloader_runCpu:150: CPU r5f1-0  is initialized to 800000000 Hz !!!
        INFO: Bootloader_runCpu:150: CPU r5f1-1 is initialized to 800000000 Hz !!!
        INFO: Bootloader_runCpu:150: CPU m4f0-0 is initialized to 400000000 Hz !!!
        INFO: Bootloader_runCpu:150: CPU a530-0 is initialized to 800000000 Hz !!!
        INFO: Bootloader_runCpu:150: CPU a530-1 is initialized to 800000000 Hz !!!
        INFO: Bootloader_loadSelfCpu:202: CPU r5f0-0 is initialized to 800000000 Hz !!!
        INFO: Bootloader_loadSelfCpu:202: CPU r5f0-1 is initialized to 800000000 Hz !!!
        INFO: Bootloader_runSelfCpu:219: All done, reseting self ...

        [DPL] Hwi post ...
        [DPL] Hwi post ... DONE !!!
        [DPL] Sleep for 100 msecs ...
        [DPL] Sleep ... DONE (Measured time = 100002 usecs, CPU cycles = 438501 ) !!!
        [DPL] Running cache operations ...
        [DPL] Running cache operations ... DONE !!!
        [DPL] Heap free size = 1984 bytes
        [DPL] Allocated 1023 bytes @ 0x70086080, heap free size = 896 bytes
        [DPL] Free'ed 1023 bytes @ 0x70086080, heap free size = 1984 bytes
        All tests have passed!!

- Congratulations now the AM64X-SK is flashed with your application and you dont need CCS anymore to run the application.
\endcond

\cond SOC_AM243X
## Flashing the application

### AM243X-LP
- **POWER-OFF** the AM243X-LP

- Set boot mode to UART BOOTMODE as shown in below image

  \imageStyle{lp_boot_pins_uart_mode.png,width:30%}
  \image html lp_boot_pins_uart_mode.png "UART BOOT MODE"

- **POWER-ON** the AM243X-LP

- You should see character "C" getting printed on the UART terminal every 2-3 seconds as shown below

  \imageStyle{lp_uart_rom_boot.png,width:80%}
  \image html lp_uart_rom_boot.png "UART output in UART BOOT MODE"

- Close the UART terminal as shown below. This is important, else the UART script in next step wont be able to connect to the UART port.

  \imageStyle{lp_ccs_uart_close.png,width:80%}
  \image html lp_ccs_uart_close.png "Close UART terminal"

- Open a command prompt and run the below command to flash the SOC initialization binary to the AM243X-LP.

        cd ${SDK_INSTALL_PATH}/tools/boot
        python uart_uniflash.py -p COM148 --cfg=sbl_prebuilt/@VAR_LP_BOARD_NAME_LOWER/default_sbl_ospi.cfg

  - Here COM148 is the port name of the identified UART port in Windows.
  - On Linux,
    - The name for UART port is typically something like `/dev/ttyUSB0`
    - On some Linux systems, one needs to use `python3` to invoke python3.x, just `python` command may invoke python 2.x which will not work with the flashing script.

- When the flashing is in progress you will see something like below

  \imageStyle{lp_flash_soc_init_in_progress.png,width:100%}
  \image html lp_flash_soc_init_in_progress.png "Flash in progress"

- After all the flashing is done, you will see something like below

        C:/ti/mcu_plus_sdk/tools/boot>python uart_uniflash.py -p COM148 --cfg=sbl_prebuilt/@VAR_LP_BOARD_NAME_LOWER/default_sbl_ospi.cfg

        Parsing config file ...
        Parsing config file ... SUCCESS. Found 3 command(s) !!!

        Executing command 1 of 3 ...
        Found flash writer ... sending sbl_prebuilt/@VAR_LP_BOARD_NAME_LOWER/sbl_uart_uniflash.release.tiimage
        Sent flashwriter sbl_prebuilt/@VAR_LP_BOARD_NAME_LOWER/sbl_uart_uniflash.release.tiimage of size 289169 bytes in 28.52s.

        Executing command 2 of 3 ...
        Command arguments : --file=sbl_prebuilt/@VAR_LP_BOARD_NAME_LOWER/sbl_ospi.release.tiimage --operation=flash --flash-offset=0x0
        Sent sbl_prebuilt/@VAR_LP_BOARD_NAME_LOWER/sbl_ospi.release.tiimage of size 241937 bytes in 25.31s.
        [STATUS] SUCCESS !!!

        Executing command 3 of 3 ...
        Command arguments : --file=C:/Users/XYZ/workspace_v10/hello_world_{board}_r5fss0-0_freertos_ti-arm-clang/Release/hello_world_{board}_r5fss0-0_freertos_ti-arm-clang.appimage --operation=flash --flash-offset=0x80000
        Sent C:/Users/XYZ/workspace_v10/hello_world_{board}_r5fss0-0_freertos_ti-arm-clang/Release/hello_world_{board}_r5fss0-0_freertos_ti-arm-clang.appimage of size 58772 bytes in 8.43s.
        [STATUS] SUCCESS !!!

        All commands from config file are executed !!!

- If flashing has failed, see \ref TOOLS_FLASH_ERROR_MESSAGES, and resolve the errors.

- If flashing is successful, do the next steps ...

### AM243X-EVM
- **POWER-OFF** the EVM

- Set boot mode to UART BOOTMODE as shown in below image

  \imageStyle{boot_pins_uart_mode.png,width:30%}
  \image html boot_pins_uart_mode.png "UART BOOT MODE"

- **POWER-ON** the EVM

- You should see character "C" getting printed on the UART terminal every 2-3 seconds as shown below

  \imageStyle{uart_rom_boot.png,width:80%}
  \image html uart_rom_boot.png "UART output in UART BOOT MODE"

- Close the UART terminal as shown below. This is important, else the UART script in next step wont be able to connect to the UART port.

  \imageStyle{ccs_uart_close.png,width:80%}
  \image html ccs_uart_close.png "Close UART terminal"

- Open a command prompt and run the below command to flash the SOC initialization binary to the EVM.

        cd ${SDK_INSTALL_PATH}/tools/boot
        python uart_uniflash.py -p COM13 --cfg=sbl_prebuilt/@VAR_BOARD_NAME_LOWER/default_sbl_ospi.cfg

  - Here COM13 is the port name of the identified UART port in Windows.
  - On Linux,
    - The name for UART port is typically something like `/dev/ttyUSB0`
    - On some Linux systems, one needs to use `python3` to invoke python3.x, just `python` command may invoke python 2.x which will not work with the flashing script.

- When the flashing is in progress you will see something like below

  \imageStyle{flash_soc_init_in_progress.png,width:100%}
  \image html flash_soc_init_in_progress.png "Flash in progress"

- After all the flashing is done, you will see something like below

        C:/ti/mcu_plus_sdk/tools/boot>python uart_uniflash.py -p COM13 --cfg=sbl_prebuilt/@VAR_BOARD_NAME_LOWER/default_sbl_ospi.cfg

        Parsing config file ...
        Parsing config file ... SUCCESS. Found 3 command(s) !!!

        Executing command 1 of 3 ...
        Found flash writer ... sending sbl_prebuilt/@VAR_BOARD_NAME_LOWER/sbl_uart_uniflash.release.tiimage
        Sent flashwriter sbl_prebuilt/@VAR_BOARD_NAME_LOWER/sbl_uart_uniflash.release.tiimage of size 289169 bytes in 28.52s.

        Executing command 2 of 3 ...
        Command arguments : --file=sbl_prebuilt/@VAR_BOARD_NAME_LOWER/sbl_ospi.release.tiimage --operation=flash --flash-offset=0x0
        Sent sbl_prebuilt/@VAR_BOARD_NAME_LOWER/sbl_ospi.release.tiimage of size 241937 bytes in 25.31s.
        [STATUS] SUCCESS !!!

        Executing command 3 of 3 ...
        Command arguments : --file=C:/Users/XYZ/workspace_v10/hello_world_{board}_r5fss0-0_freertos_ti-arm-clang/Release/hello_world_{board}_r5fss0-0_freertos_ti-arm-clang.appimage --operation=flash --flash-offset=0x80000
        Sent C:/Users/XYZ/workspace_v10/hello_world_{board}_r5fss0-0_freertos_ti-arm-clang/Release/hello_world_{board}_r5fss0-0_freertos_ti-arm-clang.appimage of size 58772 bytes in 8.43s.
        [STATUS] SUCCESS !!!

        All commands from config file are executed !!!

- If flashing has failed, see \ref TOOLS_FLASH_ERROR_MESSAGES, and resolve the errors.

- If flashing is successful, do the next steps ...

## Running the flashed application

### AM243X-LP
- **POWER-OFF** the AM243X-LP

- Switch the AM243X-LP boot mode to OSPI mode as shown below,

  \imageStyle{lp_boot_pins_ospi_mode.png,width:30%}
  \image html lp_boot_pins_ospi_mode.png "OSPI BOOT MODE"

- Re-connect the UART terminal in CCS window as shown in \ref CCS_UART_TERMINAL

- **POWER-ON** the AM243X-LP

- You should see the application output in UART terminal as below

        Starting OSPI Bootloader ...

        DMSC Firmware Version 21.1.1--v2021.01a (Terrific Lla
        DMSC Firmware revision 0x15
        DMSC ABI revision 3.1

        INFO: Bootloader_runCpu:151: CPU m4f0-0 is initialized to 400000000 Hz !!!
        INFO: Bootloader_runCpu:151: CPU r5f1-0  is initialized to 800000000 Hz !!!
        INFO: Bootloader_runCpu:151: CPU r5f1-1 is initialized to 800000000 Hz !!!
        INFO: Bootloader_loadSelfCpu:214: CPU r5f0-0 is initialized to 800000000 Hz !!!
        INFO: Bootloader_loadSelfCpu:214: CPU r5f0-1 is initialized to 800000000 Hz !!!
        INFO: Bootloader_loadSelfCpu:235: All done, reseting self ...

        [DPL] Hwi post ...
        [DPL] Hwi post ... DONE !!!
        [DPL] Sleep for 100 msecs ...
        [DPL] Sleep ... DONE (Measured time = 100002 usecs, CPU cycles = 438501 ) !!!
        [DPL] Running cache operations ...
        [DPL] Running cache operations ... DONE !!!
        [DPL] Heap free size = 1984 bytes
        [DPL] Allocated 1023 bytes @ 0x70086080, heap free size = 896 bytes
        [DPL] Free'ed 1023 bytes @ 0x70086080, heap free size = 1984 bytes
        All tests have passed!!

- Congratulations now the AM243X-LP is flashed with your application and you dont need CCS anymore to run the application.

### AM243X-EVM
- **POWER-OFF** the EVM

- Switch the EVM boot mode to OSPI mode as shown below,

  \imageStyle{boot_pins_ospi_mode.png,width:30%}
  \image html boot_pins_ospi_mode.png "OSPI BOOT MODE"

- Re-connect the UART terminal in CCS window as shown in \ref CCS_UART_TERMINAL

- **POWER-ON** the EVM

- You should see the application output in UART terminal as below

        Starting OSPI Bootloader ...

        DMSC Firmware Version 21.1.1--v2021.01a (Terrific Lla
        DMSC Firmware revision 0x15
        DMSC ABI revision 3.1

        INFO: Bootloader_runCpu:151: CPU m4f0-0 is initialized to 400000000 Hz !!!
        INFO: Bootloader_runCpu:151: CPU r5f1-0  is initialized to 800000000 Hz !!!
        INFO: Bootloader_runCpu:151: CPU r5f1-1 is initialized to 800000000 Hz !!!
        INFO: Bootloader_loadSelfCpu:214: CPU r5f0-0 is initialized to 800000000 Hz !!!
        INFO: Bootloader_loadSelfCpu:214: CPU r5f0-1 is initialized to 800000000 Hz !!!
        INFO: Bootloader_loadSelfCpu:235: All done, reseting self ...

        [DPL] Hwi post ...
        [DPL] Hwi post ... DONE !!!
        [DPL] Sleep for 100 msecs ...
        [DPL] Sleep ... DONE (Measured time = 100002 usecs, CPU cycles = 438501 ) !!!
        [DPL] Running cache operations ...
        [DPL] Running cache operations ... DONE !!!
        [DPL] Heap free size = 1984 bytes
        [DPL] Allocated 1023 bytes @ 0x70086080, heap free size = 896 bytes
        [DPL] Free'ed 1023 bytes @ 0x70086080, heap free size = 1984 bytes
        All tests have passed!!

- Congratulations now the EVM is flashed with your application and you dont need CCS anymore to run the application.

\endcond

\cond SOC_AM273X
## Flashing the application

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
        python uart_uniflash.py -p COM<x> --cfg=sbl_prebuilt/@VAR_BOARD_NAME_LOWER/default_sbl_qspi.cfg

  - Here COM<x> is the port name of the identified UART port in Windows.
  - On Linux,
    - The name for UART port is typically something like `/dev/ttyUSB0`
    - On some Linux systems, one needs to use `python3` to invoke python3.x, just `python` command may invoke python 2.x which will not work with the flashing script.

- When the flashing is in progress you will see something like below

  \imageStyle{flash_qspi_boot_in_progress.png,width:80%}
  \image html flash_qspi_boot_in_progress.png "Flash in progress"

- After all the flashing is done, you will see something like below

  \imageStyle{flash_qspi_boot_success.png,width:80%}
  \image html flash_qspi_boot_success.png "Flashing successful"

- If flashing has failed, see \ref TOOLS_FLASH_ERROR_MESSAGES, and resolve the errors.

- If flashing is successful, do the next steps ...

## Running the flashed application

- **POWER-OFF** the EVM

- Change the boot mode to \ref BOOTMODE_QSPI

- **POWER-ON** the EVM

- Re-connect the UART terminal in CCS window as shown in \ref CCS_UART_TERMINAL

- Press the reset button on the board as shown in \ref EVM_CABLES

- You should see output like below on the UART terminal

        Starting QSPI Bootloader ...
        [BOOTLOADER PROFILE] System_init                      :        483us
        [BOOTLOADER PROFILE] Drivers_open                     :         12us
        [BOOTLOADER PROFILE] Board_driversOpen                :       2715us
        [BOOTLOADER PROFILE] CPU load                         :      49459us
        [BOOTLOADER_PROFILE] SBL Total Time Taken             :      52670us

        Image loading done, switching to application ...
        [DPL] Hwi post ...
        [DPL] Hwi post ... DONE !!!
        [DPL] Sleep for 100 msecs ...
        [DPL] Sleep ... DONE (Measured time = 99752 usecs, CPU cycles = 71849 ) !!!
        [DPL] Running cache operations ...
        [DPL] Running cache operations ... DONE !!!
        [DPL] Heap free size = 1984 bytes
        [DPL] Allocated 1023 bytes @ 0x10266080, heap free size = 896 bytes
        [DPL] Free'ed 1023 bytes @ 0x10266080, heap free size = 1984 bytes
        All tests have passed!!

- Congratulations now the AM273X-EVM is flashed with your application and you dont need CCS anymore to run the application.

\endcond


\cond SOC_AM263X
## Flashing the application

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
        python uart_uniflash.py -p COM<x> --cfg=sbl_prebuilt/@VAR_BOARD_NAME_LOWER/default_sbl_qspi.cfg

  - Here COM<x> is the port name of the identified UART port in Windows.
  - On Linux,
    - The name for UART port is typically something like `/dev/ttyUSB0`
    - On some Linux systems, one needs to use `python3` to invoke python3.x, just `python` command may invoke python 2.x which will not work with the flashing script.

- When the flashing is in progress you will see something like below

  \imageStyle{flash_qspi_boot_in_progress.png,width:80%}
  \image html flash_qspi_boot_in_progress.png "Flash in progress"

- After all the flashing is done, you will see something like below

  \imageStyle{flash_qspi_boot_success.png,width:80%}
  \image html flash_qspi_boot_success.png "Flashing successful"

- If flashing has failed, see \ref TOOLS_FLASH_ERROR_MESSAGES, and resolve the errors.

- If flashing is successful, do the next steps ...

## Running the flashed application

- **POWER-OFF** the EVM

- Change the boot mode to \ref BOOTMODE_QSPI

- **POWER-ON** the EVM

- Re-connect the UART terminal in CCS window as shown in \ref CCS_UART_TERMINAL

- Press the reset button on the board as shown in \ref EVM_CABLES

- You should see output like below on the UART terminal

        Starting QSPI Bootloader ...
        [BOOTLOADER PROFILE] System_init                      :        162us
        [BOOTLOADER PROFILE] Drivers_open                     :         28us
        [BOOTLOADER PROFILE] Board_driversOpen                :        193us
        [BOOTLOADER PROFILE] CPU load                         :      26252us
        [BOOTLOADER_PROFILE] SBL Total Time Taken             :      26637us

        Image loading done, switching to application ...
        Hello World!

- Congratulations now the AM263X-CC is flashed with your application and you dont need CCS anymore to run the application.

\endcond

