#  Flash a Hello World example using DFU {#GETTING_STARTED_FLASH_DFU}

[TOC]

\attention Flashing a application will overwrite the SOC init application that was flashed earlier.
           So if you want to load and run from CCS again, you will need to do the SOC init steps again.
           See \ref EVM_SOC_INIT for more details.

\note See also \ref TOOLS_FLASH for more details on the flashing tools.

## Introduction

In this step we will flash the application that we have build and run using CCS to the EVM flash using 
USB DFU \ref TOOLS_FLASH_DFU_UNIFLASH
We can then boot this application without being connected to CCS via JTAG.

## Getting ready to flash the application

- A quick recap of steps done so far that are needed for the flashing to work
  - Make sure that you have properly installed **dfu-util** tool and DFU device is getting enumerated. 
	-If you are using windows make sure that generic usb drivers have been installed correctly \ref INSTALL_DFU_UTIL
  - Make sure python3 is installed as mentioned in \ref INSTALL_PYTHON3
  - Make sure you have the EVM power cable and USB cable connected as shown in \ref EVM_CABLES

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

\cond SOC_AM64X || SOC_AM243X
        ${SDK_INSTALL_PATH}/tools/boot/sbl_prebuilt/{board}/default_dfu_ospi.cfg
\endcond

- Edit below line in the config file to point to your application `.appimage` file.
  Give the absolute path to the `.appimage` file or path relative to `${SDK_INSTALL_PATH}/tools/boot`. **Make sure to use forward slash `/` in the filename path**.

        --file=../../examples/drivers/ipc/ipc_notify_echo/{board}/system_freertos_nortos/ipc_notify_echo_system.release.appimage --operation=flash --flash-offset=0x80000

- This file will additionally also list the flashing application that is run on the EVM and a OSPI flash bootloader that also
  needs to be flashed. You can keep this unchanged if you have not modified these applications.

- Save and close the config file.

\cond SOC_AM64X
## Flashing the application using USB DFU 
### AM64X-EVM
- **POWER-OFF** the EVM

- Set boot mode to DFU boot mode as shown in below image

  \imageStyle{bootpins-dfu-evm.jpg,width:30%}
  \image html bootpins-dfu-evm.jpg "DFU BOOT MODE"

- **POWER-ON** the EVM

- Open a command prompt or terminal and run the following command. To make sure that DFU device is getting enumerated. 

		$ dfu-util -l 

  \imageStyle{rom_dfu_enum.PNG,width:80%}
  \image html rom_dfu_enum.PNG "ROM DFU enumeration log"


- Open a command prompt or terminal and run the following command to flash the SOC initialization binary to the EVM.

        cd ${SDK_INSTALL_PATH}/tools/boot
        python usb_dfu_uniflash.py --cfg=sbl_prebuilt/@VAR_BOARD_NAME_LOWER/default_dfu_ospi.cfg

    - On some Linux systems, one needs to use `python3` to invoke python3.x, just `python` command may invoke python 2.x which will not work with the flashing script.

- When the flashing is in progress you will see something like below

  \imageStyle{rom_fw_boot.PNG,width:50%}
  \image html rom_fw_boot.PNG "DFU in progress"

- If flashing has failed, you can see the cause of error displayed in the dfu-util logs. 

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

		DMSC Firmware Version 8.5.3--v08.05.03 (Chill Capybar
		DMSC Firmware revision 0x8
		DMSC ABI revision 3.1

		[BOOTLOADER_PROFILE] Boot Media       : NOR SPI FLASH
		[BOOTLOADER_PROFILE] Boot Media Clock : 166.667 MHz
		[BOOTLOADER_PROFILE] Boot Image Size  : 114 KB
		[BOOTLOADER_PROFILE] Cores present    :
		m4f0-0
		r5f1-0
		r5f1-1
		r5f0-0
		r5f0-1
		[BOOTLOADER PROFILE] SYSFW init                       :      12230us
		[BOOTLOADER PROFILE] System_init                      :      25111us
		[BOOTLOADER PROFILE] Drivers_open                     :        269us
		[BOOTLOADER PROFILE] Board_driversOpen                :      21956us
		[BOOTLOADER PROFILE] Sciclient Get Version            :      10025us
		[BOOTLOADER PROFILE] CPU load                         :     195358us
		[BOOTLOADER_PROFILE] SBL Total Time Taken             :     264953us

		Image loading done, switching to application ...
		[IPC NOTIFY ECHO] Message exchange started by main core !!!
		[m4f0-0]     0.004012s : [IPC NOTIFY ECHO] Remote Core waiting for messages from main core ... !!!
		[r5f0-1]     0.002105s : [IPC NOTIFY ECHO] Remote Core waiting for messages from main core ... !!!
		[r5f0-1]     2.330708s : [IPC NOTIFY ECHO] Remote core has echoed all messages !!!
		[r5f1-0]     0.003170s : [IPC NOTIFY ECHO] Remote Core waiting for messages from main core ... !!!
		[r5f1-0]     2.332240s : [IPC NOTIFY ECHO] Remote core has echoed all messages !!!
		[r5f1-1]     0.003163s : [IPC NOTIFY ECHO] Remote Core waiting for messages from main core ... !!!
		[r5f1-1]     2.331869s : [IPC NOTIFY ECHO] Remote core has echoed all messages !!!
		[IPC NOTIFY ECHO] All echoed messages received by main core from 4 remote cores !!!
		[IPC NOTIFY ECHO] Messages sent to each core = 1000000
		[IPC NOTIFY ECHO] Number of remote cores = 4
		All tests have passed!!
		[m4f0-0]     3.611907s : [IPC NOTIFY ECHO] Remote core has echoed all messages !!!

- Congratulations now the EVM is flashed with your application and you dont need CCS anymore to run the application.

### AM64X-SK
\note SBL_DFU_UNIFLASH doesn't support AM64x-SK board. Please refer \ref GETTING_STARTED_FLASH
\endcond

\cond SOC_AM243X
## Flashing the application

### AM243X-LP
- **POWER-OFF** the AM243X-LP

- Set boot mode to DFU BOOTMODE as shown in below image

  \imageStyle{bootpins-dfu-lp.jpg,width:30%}
  \image html bootpins-dfu-lp.jpg "DFU BOOT MODE"

- **POWER-ON** the AM243X-LP

- Open a command prompt or terminal and run the following command. To make sure that DFU device is getting enumerated. 

		$ dfu-util -l 

  \imageStyle{rom_dfu_enum.PNG,width:80%}
  \image html rom_dfu_enum.PNG "ROM DFU enumeration log"


- Open a command prompt or terminal and run the following command to flash the SOC initialization binary to the EVM.

        cd ${SDK_INSTALL_PATH}/tools/boot
        python usb_dfu_uniflash.py --cfg=sbl_prebuilt/@VAR_BOARD_NAME_LOWER/default_dfu_ospi.cfg

    - On some Linux systems, one needs to use `python3` to invoke python3.x, just `python` command may invoke python 2.x which will not work with the flashing script.

- When the flashing is in progress you will see something like below

  \imageStyle{rom_fw_boot.PNG,width:50%}
  \image html rom_fw_boot.PNG "DFU in progress"

- If flashing has failed, you can see the cause of error displayed in the dfu-util logs. 

- If flashing is successful, do the next steps ...

### AM243X-EVM
- **POWER-OFF** the EVM

- Set boot mode to DFU boot mode as shown in below image

  \imageStyle{bootpins-dfu-evm.jpg,width:30%}
  \image html bootpins-dfu-evm.jpg "ROM DFU BOOT MODE"

- **POWER-ON** the EVM

- Open a command prompt or terminal and run the following command. To make sure that DFU device is getting enumerated. 

		$ dfu-util -l 

  \imageStyle{rom_dfu_enum.PNG,width:80%}
  \image html rom_dfu_enum.PNG "ROM DFU enumeration log"


- Open a command prompt or terminal and run the following command to flash the SOC initialization binary to the EVM.

        cd ${SDK_INSTALL_PATH}/tools/boot
        python usb_dfu_uniflash.py --cfg=sbl_prebuilt/@VAR_BOARD_NAME_LOWER/default_dfu_ospi.cfg

    - On some Linux systems, one needs to use `python3` to invoke python3.x, just `python` command may invoke python 2.x which will not work with the flashing script.

- When the flashing is in progress you will see something like below

  \imageStyle{rom_fw_boot.PNG,width:50%}
  \image html rom_fw_boot.PNG "DFU in progress"

- If flashing has failed, you can see the cause of error displayed in the dfu-util logs. 

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

		DMSC Firmware Version 8.5.3--v08.05.03 (Chill Capybar
		DMSC Firmware revision 0x8
		DMSC ABI revision 3.1

		[BOOTLOADER_PROFILE] Boot Media       : NOR SPI FLASH
		[BOOTLOADER_PROFILE] Boot Media Clock : 166.667 MHz
		[BOOTLOADER_PROFILE] Boot Image Size  : 114 KB
		[BOOTLOADER_PROFILE] Cores present    :
		m4f0-0
		r5f1-0
		r5f1-1
		r5f0-0
		r5f0-1
		[BOOTLOADER PROFILE] SYSFW init                       :      12230us
		[BOOTLOADER PROFILE] System_init                      :      25111us
		[BOOTLOADER PROFILE] Drivers_open                     :        269us
		[BOOTLOADER PROFILE] Board_driversOpen                :      21956us
		[BOOTLOADER PROFILE] Sciclient Get Version            :      10025us
		[BOOTLOADER PROFILE] CPU load                         :     195358us
		[BOOTLOADER_PROFILE] SBL Total Time Taken             :     264953us

		Image loading done, switching to application ...
		[IPC NOTIFY ECHO] Message exchange started by main core !!!
		[m4f0-0]     0.004012s : [IPC NOTIFY ECHO] Remote Core waiting for messages from main core ... !!!
		[r5f0-1]     0.002105s : [IPC NOTIFY ECHO] Remote Core waiting for messages from main core ... !!!
		[r5f0-1]     2.330708s : [IPC NOTIFY ECHO] Remote core has echoed all messages !!!
		[r5f1-0]     0.003170s : [IPC NOTIFY ECHO] Remote Core waiting for messages from main core ... !!!
		[r5f1-0]     2.332240s : [IPC NOTIFY ECHO] Remote core has echoed all messages !!!
		[r5f1-1]     0.003163s : [IPC NOTIFY ECHO] Remote Core waiting for messages from main core ... !!!
		[r5f1-1]     2.331869s : [IPC NOTIFY ECHO] Remote core has echoed all messages !!!
		[IPC NOTIFY ECHO] All echoed messages received by main core from 4 remote cores !!!
		[IPC NOTIFY ECHO] Messages sent to each core = 1000000
		[IPC NOTIFY ECHO] Number of remote cores = 4
		All tests have passed!!
		[m4f0-0]     3.611907s : [IPC NOTIFY ECHO] Remote core has echoed all messages !!!


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

		DMSC Firmware Version 8.5.3--v08.05.03 (Chill Capybar
		DMSC Firmware revision 0x8
		DMSC ABI revision 3.1

		[BOOTLOADER_PROFILE] Boot Media       : NOR SPI FLASH
		[BOOTLOADER_PROFILE] Boot Media Clock : 166.667 MHz
		[BOOTLOADER_PROFILE] Boot Image Size  : 114 KB
		[BOOTLOADER_PROFILE] Cores present    :
		m4f0-0
		r5f1-0
		r5f1-1
		r5f0-0
		r5f0-1
		[BOOTLOADER PROFILE] SYSFW init                       :      12230us
		[BOOTLOADER PROFILE] System_init                      :      25111us
		[BOOTLOADER PROFILE] Drivers_open                     :        269us
		[BOOTLOADER PROFILE] Board_driversOpen                :      21956us
		[BOOTLOADER PROFILE] Sciclient Get Version            :      10025us
		[BOOTLOADER PROFILE] CPU load                         :     195358us
		[BOOTLOADER_PROFILE] SBL Total Time Taken             :     264953us

		Image loading done, switching to application ...
		[IPC NOTIFY ECHO] Message exchange started by main core !!!
		[m4f0-0]     0.004012s : [IPC NOTIFY ECHO] Remote Core waiting for messages from main core ... !!!
		[r5f0-1]     0.002105s : [IPC NOTIFY ECHO] Remote Core waiting for messages from main core ... !!!
		[r5f0-1]     2.330708s : [IPC NOTIFY ECHO] Remote core has echoed all messages !!!
		[r5f1-0]     0.003170s : [IPC NOTIFY ECHO] Remote Core waiting for messages from main core ... !!!
		[r5f1-0]     2.332240s : [IPC NOTIFY ECHO] Remote core has echoed all messages !!!
		[r5f1-1]     0.003163s : [IPC NOTIFY ECHO] Remote Core waiting for messages from main core ... !!!
		[r5f1-1]     2.331869s : [IPC NOTIFY ECHO] Remote core has echoed all messages !!!
		[IPC NOTIFY ECHO] All echoed messages received by main core from 4 remote cores !!!
		[IPC NOTIFY ECHO] Messages sent to each core = 1000000
		[IPC NOTIFY ECHO] Number of remote cores = 4
		All tests have passed!!
		[m4f0-0]     3.611907s : [IPC NOTIFY ECHO] Remote core has echoed all messages !!!

- Congratulations now the EVM is flashed with your application and you dont need CCS anymore to run the application.

\endcond

