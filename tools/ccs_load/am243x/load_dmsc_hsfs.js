/*
 * Copyright (c) 2021, Texas Instruments Incorporated
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
Description:
   Load the DMSC firmware and initialize DMSC with a default board configuration from R5F.

Usage:

1. Modify "sdkPath" (search for EDIT THIS) to point to the absolute path of the SDK.
    - On windows make sure to use '/' or '\\' as path separator

2. Launch AM243x target connection in CCS, however DO NOT connect to any CPUs.

3. From CCS Scripting console do (CCS Tool Bar > View > Scripting Console)
    js:> loadJSFile "<path/to/sdk>/tools/ccs_load/am243x/load_dmsc_hsfs.js"

4. After successful execution you should see a log like below

  On the CCS scripting console in CCS,

    js:> LoadJSFile "C:/ti/mcu_plus_sdk/tools/ccs_load/am243x/load_dmsc_hsfs.js"
    Connecting to MCU Cortex_R5_0!
    Writing While(1) for R5F
    Running the board configuration initialization from R5!
    Happy Debugging

    js:>

  On the am243x.ccxml:CIO console in CCS, i.e the R5F console,

    [MAIN_Cortex_R5_0_0]
    DMSC Firmware Version AA.B.C-vDDDD.EEE-FF-gggggg (HHH
    DMSC Firmware revision 0xNN
    DMSC ABI revision x.y

    [SCICLIENT] ABI check PASSED
    [SCICLIENT] Board Configuration with Debug enabled ...
    [SCICLIENT] Common Board Configuration PASSED
    [SCICLIENT] PM Board Configuration PASSED
    [SCICLIENT] RM Board Configuration PASSED
    [SCICLIENT] Security Board Configuration PASSED

    DMSC Firmware Version AA.B.C-vDDDD.EEE-FF-gggggg (HHH
    DMSC Firmware revision 0xNN
    DMSC ABI revision x.y

    All tests have passed!!


5. If any of the logs in step 4 show "fail" or "error" messages then
   check your EVM, CCS, SDK setup and try again.
*/

function updateScriptVars()
{
    //Open a debug session
    dsMCU1_0 = debugServer.openSession( ".*MAIN_Cortex_R5_0_0" );
}

function connectTargets()
{
    /* Set timeout of 20 seconds */
    script.setScriptTimeout(20000);
    updateScriptVars();

    //Init_M4()
    print("Connecting to MCU Cortex_R5_0!");

    // Connect the MCU R5F
    dsMCU1_0.target.connect();

    dsMCU1_0.memory.fill(0x78000000, 0, 0x2000, 0);
    print("Writing While(1) for R5F")
    dsMCU1_0.memory.writeWord(0, 0x78000000, 0xE59FF004); /* ldr        pc, [pc, #4] */
    dsMCU1_0.memory.writeWord(0, 0x78000004, 0x38);       /* Address 0x38 */
    dsMCU1_0.memory.writeWord(0, 0x78000038, 0xEAFFFFFE) /* b          #0x38 */

    // Halt the R5F and re-run.
    dsMCU1_0.target.halt();

    // Reset the R5F and run.
    dsMCU1_0.target.reset();

    print("Running the board configuration initialization from R5!");
    // Load the board configuration init file.
    dsMCU1_0.memory.loadProgram(ccs_init_elf_file);
    // Halt the R5F and re-run.
    dsMCU1_0.target.halt();
    // Run Synchronously for the executable to finish
    dsMCU1_0.target.run();

    return 0;
}

function disconnectTargets()
{
    updateScriptVars();
    // Reset the R5F to be in clean state.
    dsMCU1_0.target.reset();
}

function doEverything()
{
    var run = true;

    if(!File(ccs_init_elf_file).isFile())
    {
        print("[ERROR] File "+ccs_init_elf_file+" not found !!!");
        run = false;
    }

    if(run == true)
    {
        updateScriptVars();
        var connectSuccess = connectTargets();
        if(connectSuccess == 0)
        {
            disconnectTargets();
            print("Happy Debugging!!");
        }
    }
    else
    {
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

if (!withinCCS)
{
    // !!! EDIT THIS !!! Add absolute path to SDK in your environment variables
    // OR set this variable to the absolute path of the SDK
    sdkPath = "C:/ti/mcu_plus_sdk";
}
else
{
    sdkPath = System.getenv("MCU_PLUS_SDK_AM243X_PATH");
    if(sdkPath == null)
    {
        // !!! EDIT THIS !!! Add absolute path to SDK in your environment variables
        // OR set this variable to the absolute path of the SDK
        sdkPath = "C:/ti/mcu_plus_sdk";
    }
}

// path to board config elf
ccs_init_elf_file = sdkPath+"/tools/ccs_load/am243x/sciclient_ccs_init.release.out";

// !!! EDIT THIS !!! Add absolute path to the CCXML file here.
fileCcxml = "C:/ti/AM243x.ccxml"

// Create scripting environment and get debug server if running standalone
if (!withinCCS)
{
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
    if(!File(fileCcxml).isFile())
    {
        print("[ERROR] File "+fileCcxml+" not found !!!");
        print("Seems like the script is not run from within CCS. Please edit the load_dmsc_hsfs.js script to add a path to your CCXML configuration file in this case.")
    }
    else
    {
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


