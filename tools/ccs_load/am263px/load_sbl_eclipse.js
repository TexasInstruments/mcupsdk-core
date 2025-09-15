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

2. Launch AM263Px target connection in CCS, however DO NOT connect to any CPUs.

3. From CCS Scripting console do (CCS Tool Bar > View > Scripting Console)
    js:> loadJSFile "<path/to/sdk>/tools/ccs_load/am263px/load_sbl_eclipse.js"

4. After successful execution you should see a log like below

  On the CCS scripting console in CCS,

    js:> LoadJSFile "/home/abishekss/workarea/mcupsdk/mcu_plus_sdk/tools/ccs_load/am263px/load_sbl_eclipse.js"
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
    var resetType = dsMCU1_0.target.getResetType(1);
    print("Going to issue reset: '" + resetType.getName() + "' (" + resetType.getDescription() + ") ...");
    resetType.issueReset();
}

function connectTargets() {
    /* Set timeout of 10 seconds */
    script.setScriptTimeout(10000);
    updateScriptVars();

    //Writes 15 in MSS_L2_MEM_INIT
    dsMCU1_0.target.connect();
    dsMCU1_0.memory.writeWord(0, 0x50D00240, 0xF)

    print("[Cortex_R5_0] L2 Memory Init Done ...");

    // Connect the MCU R5F
    connectHaltResetCpu()

    print("[Cortex_R5_0] Loading SBL Init Code ... ");
    dsMCU1_0.target.halt();
    dsMCU1_0.memory.loadRaw(0, 0x70002000, sbl_bin_file, 32, false);

    // Copy 640 bytes data from MSRAM_VECS to R5F_VECS
    print("[Cortex_R5_0] Copying data to R5F_VECS ... ");
    data = dsMCU1_0.memory.readData(0, 0x70002000, 32, 160)
    for (i in data) {
        dsMCU1_0.memory.writeData(0, 0x00020000 + i * 4, data[i], 32)
    }

    print("[Cortex_R5_0] Triggering ROM Eclipse ... ");
    /* Trigger ROM eclipse */
    dsMCU1_0.memory.writeWord(0, 0x50D00080, 0x7);

    print("[Cortex_R5_0] Loading SBL ... ");
    dsMCU1_0.memory.loadProgram(sbl_elf_file);

    print("[Cortex_R5_0] Running SBL ... ");
    dsMCU1_0.target.runAsynch();

    return 0;
}

function doEverything() {
    var run = true;

    if (!File(sbl_elf_file).isFile()) {
        print("[ERROR] File " + sbl_elf_file + " not found !!!");
        run = false;
    }

    if (!File(sbl_bin_file).isFile()) {
        print("[ERROR] File " + sbl_bin_file + " not found !!!");
        run = false;
    }

    if (run == true) {
        updateScriptVars();
        var connectSuccess = connectTargets();
        if (connectSuccess == 0) {
            print("Happy Debugging!!");
        }
    }
    else {
        print("Please read the instructions at top of this file to make sure the paths to the SDK are set correctly !!!")
    }
}

// Import the DSS packages into our namespace to save on typing
importPackage(Packages.com.ti.debug.engine.scripting)
importPackage(Packages.com.ti.ccstudio.scripting.environment)
importPackage(Packages.java.lang)
importPackage(java.io);
importPackage(java.lang);

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
    sdkPath = System.getenv("MCU_PLUS_SDK_AM263PX_PATH");
    if (sdkPath == null) {
        // !!! EDIT THIS !!! Add absolute path to SDK in your environment variables
        // OR set this variable to the absolute path of the SDK
        sdkPath = "C:/ti/mcu_plus_sdk";
    }
}

// path to sbl elf
sbl_elf_file = sdkPath + "/examples/drivers/boot/sbl_null/am263px-cc/r5fss0-0_nortos/ti-arm-clang/sbl_null.release.out";

// path to sbl bin
sbl_bin_file = sdkPath + "/examples/drivers/boot/sbl_null/am263px-cc/r5fss0-0_nortos/ti-arm-clang/sbl_null.release.bin"

// !!! EDIT THIS !!! Add absolute path to the CCXML file here.
fileCcxml = "C:/ti/AM263Px.ccxml"

// Create scripting environment and get debug server if running standalone
if (!withinCCS) {
    // Import the DSS packages into our namespace to save on typing
    importPackage(Packages.com.ti.debug.engine.scripting);
    importPackage(Packages.com.ti.ccstudio.scripting.environment);
    importPackage(Packages.java.lang);

    // Create our scripting environment object - which is the main entry point into any script and
    // the factory for creating other Scriptable ervers and Sessions
    script = ScriptingEnvironment.instance();

    // Get the Debug Server and start a Debug Session
    debugServer = script.getServer("DebugServer.1");

    // Check if the CCXML file exists.
    if (!File(fileCcxml).isFile()) {
        print("[ERROR] File " + fileCcxml + " not found !!!");
        print("Seems like the script is not run from within CCS. Please edit the load_sbl_eclipse.js script to add a path to your CCXML configuration file in this case.")
    }
    else {
        debugServer.setConfig(fileCcxml);
        doEverything();
    }
}
else // otherwise leverage existing scripting environment and debug server
{
    debugServer = ds;
    script = env;
    doEverything();
}