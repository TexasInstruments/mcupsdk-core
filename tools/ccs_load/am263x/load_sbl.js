/*
 * Copyright (c) 2022-23 Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 Description: Script used to load SBL executables to R5 Core-0 on the SOC.

 Usage:

1. Modify "sdkPath" (search for EDIT THIS) to point to the absolute path of the SDK.
    - On windows make sure to use '/' or '\\' as path separator

2. Launch AM263x target connection in CCS, however DO NOT connect to any CPUs.

3. From CCS Scripting console do (CCS Tool Bar > View > Scripting Console)
    Upload this script to the console

4. After successful execution you should see a log like below

  On the CCS scripting console in CCS,

    js:> 
    [Cortex_R5_0] L2 Memory Init Done ...
    Going to issue reset: 'System Reset' (This is a System-Level Warm Reset) ...
    [Cortex_R5_0] Loading SBL Init Code ...
    [Cortex_R5_0] Copying data to R5F_VECS ...
    [Cortex_R5_0] Triggering ROM Eclipse ...
    [Cortex_R5_0] Loading SBL ...
    [Cortex_R5_0] Running SBL ...
    Happy Debugging!!

    js:>

5. If any of the logs in step 4 show "fail" or "error" messages then
   check your EVM, CCS, SDK setup and try again.
*/

const fs = require("fs");
function updateScriptVars() {
    //Open a debug session
    dsMCU1_0 = debugServer.openSession(".*Cortex_R5_0*");
}

function disconnectTargets() {
    updateScriptVars();
    // Reset the R5F to be in clean state.
    dsMCU1_0.target.reset();
}

function wait(ms) {
    var start = new Date().getTime();
    var end = start;
    while (end < start + ms) {
        end = new Date().getTime();
    }
}

function connectHaltResetCpu() {
    dsMCU1_0.target.halt();
    var resetType = dsMCU1_0.target.getResets();
    var allowedResets = Object.keys(resetType).filter((reset) => resetType[reset]);
    console.log("Going to issue reset: " + allowedResets[1]);
    dsMCU1_0.target.reset(allowedResets[1]);
}

function connectTargets() {
    /* Set timeout of 10 seconds */
    debugServer.setScriptingTimeout(10000);
    updateScriptVars();

    // Writes 15 in MSS_L2_MEM_INIT
    dsMCU1_0.target.connect();
    dsMCU1_0.memory.write(0x50D00240, 0xF, 32);

    console.log("[Cortex_R5_0] L2 Memory Init Done ...");

    // Connect the MCU R5F
    connectHaltResetCpu()

    console.log("[Cortex_R5_0] Loading SBL Init Code ... ");
    dsMCU1_0.target.halt();
    dsMCU1_0.memory.loadBinary( 0x70002000, sbl_bin_file);

    // Copy 640 bytes data from MSRAM_VECS to R5F_VECS
    console.log("[Cortex_R5_0] Copying data to R5F_VECS ... ");
    data = dsMCU1_0.memory.read(0x70002000, 160, 32)
    for (i in data) {
        dsMCU1_0.memory.write(0x00020000 + i * 4, data[i], 32)
    }

    console.log("[Cortex_R5_0] Triggering ROM Eclipse ... ");
    /* Trigger ROM eclipse */
    dsMCU1_0.memory.write(0x50D00080, 0x7, 32);

    console.log("[Cortex_R5_0] Loading SBL ... ");
    dsMCU1_0.memory.loadProgram(sbl_elf_file);

    console.log("[Cortex_R5_0] Running SBL ... ");
    dsMCU1_0.target.run(false);

    return 0;
}

function doEverything() {
    var run = true;

    if (!fs.existsSync(sbl_elf_file) || !fs.statSync(sbl_elf_file).isFile()) {
        console.log("[ERROR] File " + sbl_elf_file + " not found !!!");
        run = false;
    }

    if (!fs.existsSync(sbl_bin_file) || !fs.statSync(sbl_bin_file).isFile()) {
        console.log("[ERROR] File " + sbl_bin_file + " not found !!!");
        run = false;
    }

    if (run == true) {
        updateScriptVars();
        var connectSuccess = connectTargets();
        if (connectSuccess == 0) {
            console.log("Happy Debugging!!");
        }
    }
    else {
        console.log("Please read the instructions at top of this file to make sure the paths to the SDK are set correctly !!!")
    }
}


var ds;
var debugServer;
var script;

// Check to see if running from within CCSv4 Scripting Console
var withinCCS = (ds !== undefined);

var sdkPath = null;

if (!withinCCS) {
    // !!! EDIT THIS !!! Add absolute path to SDK in your environment variables
    // OR set this variable to the absolute path of the SDK
    sdkPath = "C:/ti/mcu_plus_sdk";
}
else {
    sdkPath = System.getenv("MCU_PLUS_SDK_AM263X_PATH");
    if (sdkPath == null) {
        // !!! EDIT THIS !!! Add absolute path to SDK in your environment variables
        // OR set this variable to the absolute path of the SDK
        sdkPath = "C:/ti/mcu_plus_sdk";
    }
}

// path to sbl elf
sbl_elf_file = sdkPath + "/examples/drivers/boot/sbl_null/am263x-cc/r5fss0-0_nortos/ti-arm-clang/sbl_null.release.out";

// path to sbl bin
sbl_bin_file = sdkPath + "/examples/drivers/boot/sbl_null/am263x-cc/r5fss0-0_nortos/ti-arm-clang/sbl_null.release.bin"

// !!! EDIT THIS !!! Add absolute path to the CCXML file here.
fileCcxml = "C:/ti/AM263x.ccxml"

// Create scripting environment and get debug server if running standalone
if (!withinCCS) {
    const ds = initScripting();

    // Check if the CCXML file exists.
    if (!fs.existsSync(fileCcxml) || !fs.statSync(fileCcxml).isFile()) {
        console.log("[ERROR] File " + fileCcxml + " not found !!!");
        console.log("Seems like the script is not run from within CCS. Please edit the load_sbl.js script to add a path to your CCXML configuration file in this case.")
    }
    else {
        debugServer.configure(fileCcxml);
        doEverything();
    }
}
else // otherwise leverage existing scripting environment and debug server
{
    debugServer = ds;
    doEverything();
}
