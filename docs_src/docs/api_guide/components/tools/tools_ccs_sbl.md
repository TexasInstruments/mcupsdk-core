# CCS Tools {#TOOLS_CCS}

[TOC]

## Introduction

This section describes CCS utility scripts that can optionally be used to make your development more productive.

## Application loader

`load_sbl.js` script located at `${SDK_INSTALL_PATH}/tools/ccs_load/{soc}` is useful for loading SBL executables to R5 Core-0 in a single step.

### Usage

-# Search and modify below mentioned variables in the script file before loading. These are used to construct path to the example .out file
   when the example is built via makefiles. Alternatively, can you give absolute paths to the example .out of interest.

    <table>
    <tr>
        <th>Variables
        <th>Description
    </tr>
    <tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/tools/ccs_load/@VAR_SOC_NAME_LOWER/load_sbl.js</td></tr>
    <tr>
        <td>sdkPath
        <td>Points to the absolute path of the SDK. On Windows, make sure to use `/` as path separator.
    </tr>
    <tr>
        <td>sbl_elf_file
        <td>Points to the path of the sbl elf file.
    </tr>
    <tr>
        <td>sbl_bin_file
        <td>Points to the path of the sbl bin file.
    </tr>
    </table>

-# Launch the SOC target connection in CCS and do the SOC initialization, however DO NOT connect to any CPUs or load applications via CCS GUI.
   (see \ref CCS_LAUNCH_PAGE)


-# Open CCS scripting console `View> Console > Scripting Console` and upload the JS file as shown below
<br/>
        \imageStyle{load_js_file.png,width:80%}
        \image html load_js_file.png "Load JS File"

-# After successful execution you should see a log like below. Example used here is "sbl_null"

        [Cortex_R5_0] L2 Memory Init Done ...
        Going to issue reset: System Reset
        [Cortex_R5_0] Loading SBL Init Code ...
        [Cortex_R5_0] Copying data to R5F_VECS ...
        [Cortex_R5_0] Triggering ROM Eclipse ...
        [Cortex_R5_0] Loading SBL ...
        [Cortex_R5_0] Running SBL ...
        Happy Debugging!!

-# Please note that if a .out is not found, that CPU is skipped over with a information message on the console.

-# If any of the logs in step 3 show "fail" or "error" messages then
   check your EVM, CCS, SDK setup, executable path and try again.

-# To reload without power cycle, repeat step 2 onwards.

-# See also description at the top of this file `load_sbl.js` for detailed instructions
