/*
 * Copyright (c) 2024, Texas Instruments Incorporated
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
   Load executables to various CPUs on the SOC

Usage:

1. Modify "sdkPath" (search for EDIT THIS) to point to the absolute path of the SDK.
    - On windows make sure to use '/' or '\\' as path separator

2. Launch AM65x_GP_EVM target connection in CCS, however DO NOT connect to any CPUs.

3. From CCS Scripting console do (CCS Tool Bar > View > Scripting Console)

        js:> loadJSFile "<path/to/sdk>/tools/ccs_load/am65x/load.js"

4. After successful execution you should see a log like below

        js:> loadJSFile "C:/ti/mcu_plus_sdk/tools/ccs_load/am65x/load.js"
        Connecting, halting, reseting ...
        [r5fss-0] Loading ... C:/ti/mcu_plus_sdk/examples/empty/am65x-idk/r5fss0-0_freertos/ti-arm-clang/empty.release.out
        [r5fss-1] Loading ... C:/ti/mcu_plus_sdk/examples/empty/am65x-idk/r5fss0-1_freertos/ti-arm-clang/empty.release.out
        [r5fss-1] Running ...
        [r5fss-0] Running ...
        All DONE !!!

5. If any of the logs in step 4 show "fail" or "error" messages then
   check your EVM, CCS, SDK setup, Executables path and try again.

6. To reload without power cycles, repeat step 4 onwards.

*/

// !!! EDIT THIS !!! Add absolute path to SDK in your environment variables
//                   OR set this variable to the absolute path of the SDK
sdkPath = System.getenv("MCU_PLUS_SDK_AM65X_PATH");
if(sdkPath == null)
{
    sdkPath = "C:/ti/mcu_plus_sdk";
}

// !!! EDIT THIS !!! Edit all of below based on the example you want to run
runAfterLoad = true;
examplePath = "examples/empty";
exampleName = "empty";
os = "nortos";
profile = "release"
board = "am65x-idk";

// !!! EDIT THIS !!! Comment lines below for CPUs that are not needed to be loaded, change 'os' based on the 'os' combo that you have
var elf_file = {};
elf_file["r5fss-0"] = makeElfFileName("r5fss0-0", "freertos", "ti-arm-clang", exampleName, profile);
elf_file["r5fss-1"] = makeElfFileName("r5fss0-1", "freertos", "ti-arm-clang", exampleName, profile);

function makeElfFileName(cpu, os, compiler, exampleName, profile)
{
    return sdkPath + "/" + examplePath + "/" + board + "/" + cpu + "_" + os + "/" + compiler + "/" + exampleName + "." + profile + ".out"
}

function connectHaltResetCpu(cpu)
{
    dsCPU[cpu].target.connect();
    dsCPU[cpu].target.halt();
    dsCPU[cpu].target.reset();
}

function runCpu(cpu)
{
    if(cpu && File(elf_file[cpu]).isFile() )
    {
        if(runAfterLoad)
        {
            print("[" + cpu + "] Running ... ");
            dsCPU[cpu].target.runAsynch();
        }
    }
}

function loadCpu(cpu)
{
    if(cpu && File(elf_file[cpu]).isFile() )
    {
        print("[" + cpu + "] Loading ... " + elf_file[cpu] );
        dsCPU[cpu].memory.loadProgram(elf_file[cpu]);
    }
    else
    {
        print("[" + cpu + "] Skipping load, " + elf_file[cpu] + " file NOT FOUND ... " );
    }
}

function doEverything()
{
    /* Set timeout of in units of msecs */
    script.setScriptTimeout(10 * 1000);

    // Open a debug session
    dsCPU["r5fss-0"] = debugServer.openSession( ".*MCU_PULSAR_Cortex_R5_0" );
    dsCPU["r5fss-1"] = debugServer.openSession( ".*MCU_PULSAR_Cortex_R5_1" );

    print("Connecting, halting, reseting ...");
    connectHaltResetCpu("r5fss-0");
    connectHaltResetCpu("r5fss-1");

    loadCpu("r5fss-0");
    loadCpu("r5fss-1");

    runCpu("r5fss-1");

    /* run this CPU last, this is not MANDATORY
     * but in most cases, this runs the main HOST and usually is the last to start
     */
    runCpu("r5fss-0");

    print("All DONE !!!");
}

// Import the DSS packages into our namespace to save on typing
importPackage(Packages.com.ti.debug.engine.scripting)
importPackage(Packages.com.ti.ccstudio.scripting.environment)
importPackage(Packages.java.lang)
importPackage(java.io);
importPackage(java.lang);

var dsCPU = {};
var ds;
var debugServer;
var script;

// Check to see if running from within CCSv4 Scripting Console
var withinCCS = (ds !== undefined);

// !!! EDIT THIS !!! Add absolute path to the CCXML file here.
fileCcxml = "C:/ti/AM65x.ccxml"

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

