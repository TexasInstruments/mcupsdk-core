# CCS Tools {#TOOLS_CCS}

[TOC]

## Introduction

This section describes CCS utility scripts that can optionally be used to make your development more productive.

## Application loader

`load.js` script located at `${SDK_INSTALL_PATH}/tools/ccs_load/{soc}` is useful for loading and running applications across multiple CPUs in one step.

### Usage

-# Search and modify below mentioned variables in the script file before loading. These are used to construct path to the example .out file
   when the example is built via makefiles. Alternatively, can you give absolute paths to the example .out of interest.

    <table>
    <tr>
        <th>Variables
        <th>Description
    </tr>
    <tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/tools/ccs_load/@VAR_SOC_NAME_LOWER/load.js</td></tr>
    <tr>
        <td>sdkPath
        <td>Points to the absolute path of the SDK. On Windows, make sure to use `/` as path separator.
    </tr>
    <tr>
        <td>runAfterLoad
        <td>set to `true` or `false`, depending on if want to run the application after loading.
    </tr>
    <tr>
        <td>examplePath
        <td>Relative example path from `sdkPath` above, e.g. `examples/drivers/ipc/ipc_rpmsg_echo`
    </tr>
    <tr>
        <td>exampleName
        <td>Example name, e.g. `ipc_rpmsg_echo`
    <tr>
        <td>os
        <td>OS name, `nortos` or `freertos`
    </tr>
    <tr>
        <td>profile
        <td>Build profile to use, `debug` or `release`
    </tr>
    <tr>
        <td>board
        <td>Board to use. e.g. `@VAR_BOARD_NAME_LOWER`
    </tr>
    </table>

-# Launch the SOC target connection in CCS and do the SOC initialization, however DO NOT connect to any CPUs or load applications via CCS GUI.
   (see \ref CCS_LAUNCH_PAGE)

-# Open CCS scripting console `CCS Tool Bar > View > Scripting Console` and do below,

        js:> loadJSFile "C:/ti/mcu_plus_sdk/tools/ccs_load/@VAR_SOC_NAME_LOWER/load.js"

   - **NOTE** replace `C:/ti/mcu_plus_sdk` with the absolute path to where the SDK is installed.

-# After successful execution you should see a log like below. Example used here is "ipc_rpmsg_echo"

        js:> loadJSFile "C:/ti/mcu_plus_sdk/tools/ccs_load/@VAR_SOC_NAME_LOWER/load.js"
        Connecting, halting, reseting ...
        [r5fss0-0] Loading ... C:/ti/mcu_plus_sdk/examples/drivers/ipc/ipc_rpmsg_echo/@VAR_BOARD_NAME_LOWER/r5fss0-0_freertos/ti-arm-clang/ipc_rpmsg_echo.debug.out
        [r5fss0-1] Loading ... C:/ti/mcu_plus_sdk/examples/drivers/ipc/ipc_rpmsg_echo/@VAR_BOARD_NAME_LOWER/r5fss0-1_nortos/ti-arm-clang/ipc_rpmsg_echo.debug.out
        [r5fss1-0] Loading ... C:/ti/mcu_plus_sdk/examples/drivers/ipc/ipc_rpmsg_echo/@VAR_BOARD_NAME_LOWER/r5fss1-0_nortos/ti-arm-clang/ipc_rpmsg_echo.debug.out
        [r5fss1-1] Loading ... C:/ti/mcu_plus_sdk/examples/drivers/ipc/ipc_rpmsg_echo/@VAR_BOARD_NAME_LOWER/r5fss1-1_nortos/ti-arm-clang/ipc_rpmsg_echo.debug.out
        [m4fss0-0] Skipping load, C:/ti/mcu_plus_sdk/examples/drivers/ipc/ipc_rpmsg_echo/@VAR_BOARD_NAME_LOWER/m4fss0-0_nortos/ti-arm-clang/ipc_rpmsg_echo.debug.out file NOT FOUND ...
        [r5fss0-1] Running ...
        [r5fss1-0] Running ...
        [r5fss1-1] Running ...
        [r5fss0-0] Running ...
        All DONE !!!

-# Please note that if a .out is not found, that CPU is skipped over with a information message on the console.

-# If any of the logs in step 4 show "fail" or "error" messages then
   check your EVM, CCS, SDK setup, executable path and try again.

-# To reload without power cycle, repeat step 3 onwards.

-# See also description at the top of this file `load.js` for detailed instructions

\cond SOC_AM64X || SOC_AM243X
## DMSC loader

`load_dmsc.js` located at `${SDK_INSTALL_PATH}/tools/ccs_load/{soc}` is useful to perform SOC initialization using CCS scripting.
This script basically loads `sysfw.bin` to M3 core to initialize DMSC and then loads `sciclient_set_boardcfg.release.out` to R5F core to perform the default board configuration.

This script is especially useful for initial custom board bring up when only GEL files are available and nothing else.

Please refer \ref EVM_SOC_INIT_NOBOOT_MODE for how to run this script.

\endcond