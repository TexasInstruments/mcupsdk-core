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

2. Launch AM64x target connection in CCS, however DO NOT connect to any CPUs.

3. From CCS Scripting console do (CCS Tool Bar > View > Scripting Console)
    js:> loadJSFile "<path/to/sdk>/tools/ccs_load/am64x/load_dmsc.js"

4. After successful execution you should see a log like below

  On the CCS scripting console in CCS,

    js:> loadJSFile "C:/ti/mcu_plus_sdk/tools/ccs_load/am64x/load_dmsc.js"
    Connecting to DMSC_Cortex_M3_0!
    Fill R5F ATCM memory...
    Writing While(1) for R5F
    Loading DMSC Firmware ... C:/ti/mcu_plus_sdk/source/drivers/sciclient/soc/am64x_am243x/sysfw.bin
    DMSC Firmware Load Done...
    DMSC Firmware run starting now...
    Connecting to MCU Cortex_R5_0!
    Main Boot Mode is 120
    Running the board configuration initialization from R5!
    Happy Debugging!!

    js:>

  On the AM64x.ccxml:CIO console in CCS, i.e the R5F console,

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

  On the AM64x.ccxml console in CCS, i.e the GEL output console,

    DMSC_Cortex_M3_0: GEL Output: This GEL is currently only supported for use from the Cortex-M3 inside the DMSC.
    DMSC_Cortex_M3_0: GEL Output: Do not run this GEL from any other CPU on the SoC.
    DMSC_Cortex_M3_0: GEL Output: This script sets the first address translation region to [0x8000_0000, 0x0000_0000].
    DMSC_Cortex_M3_0: GEL Output: It also sets the second address translation region to    [0x6000_0000, 0x4000_0000].
    DMSC_Cortex_M3_0: GEL Output: This is consistent with the SoC DV assumptions.
    DMSC_Cortex_M3_0: GEL Output: Configuring ATCM for the R5Fs
    DMSC_Cortex_M3_0: GEL Output: ATCM Configured.
    ...
    DMSC_Cortex_M3_0: GEL Output: Powering up all PSC power domains in progress...
    DMSC_Cortex_M3_0: GEL Output: Powering up MAIN domain peripherals...
    ...
    DMSC_Cortex_M3_0: GEL Output: Powering up LPSC_A53_CLUSTER_0
    DMSC_Cortex_M3_0: GEL Output: Power domain and module state changed successfully.
    ...
    DMSC_Cortex_M3_0: GEL Output:
    DMSC_Cortex_M3_0: GEL Output: *****DDR is configured using R5 or A53 GELs
    DMSC_Cortex_M3_0: GEL Output: M4F WFI Vector set into IRAM.
    MAIN_Cortex_R5_0_0: GEL Output: Running from R5
    MAIN_Cortex_R5_0_0: GEL Output:

    DDR not initialized with R5 connect.

    Go to menu Scripts --> AM64 DDR Initialization -> AM64_DDR_Initialization_ECC_Disabled to initialize DDR.

    ====

5. If any of the logs in step 4 show "fail" or "error" messages then
   check your EVM, CCS, SDK setup and try again.
*/

function updateScriptVars()
{
    //Open a debug session
    dsMCU1_0 = debugServer.openSession( ".*MAIN_Cortex_R5_0_0" );
    dsDMSC_0 = debugServer.openSession( ".*DMSC_Cortex_M3_0" );
}

function connectTargets()
{
    /* Set timeout of 20 seconds */
    script.setScriptTimeout(200000);
    updateScriptVars();

    print("Connecting to DMSC_Cortex_M3_0!");
    // Connect targets
    dsDMSC_0.target.connect();
    // Initialize M4F
    try {
        dsDMSC_0.expression.evaluate("Init_M4()");
    } catch(e) {
        dsDMSC_0.target.disconnect();

        var gelPath = File(debugServer.getScriptingDirectory() + "/../emulation/gel/AM64x/AM64x.gel").getCanonicalPath();
        print("[ERROR] GEL files are not loaded to Cortex-M3 !!! Please choose the right target configuration or load GELs manually from "+ gelPath +" and try again...");

        return -1;
    }

    print("Fill R5F ATCM memory...");
    dsDMSC_0.memory.fill(0x78000000, 0, 0x2000, 0);
    print("Writing While(1) for R5F")
    dsDMSC_0.memory.writeWord(0, 0x78000000, 0xE59FF004); /* ldr        pc, [pc, #4] */
    dsDMSC_0.memory.writeWord(0, 0x78000004, 0x38);       /* Address 0x38 */
    dsDMSC_0.memory.writeWord(0, 0x78000038, 0xEAFFFFFE) /* b          #0x38 */

    /* RAT Config for OCSRAM SYSFW load */
    dsDMSC_0.memory.writeWord(0, 0x44200024, 0x00060000);
    dsDMSC_0.memory.writeWord(0, 0x44200028, 0x44060000);
    dsDMSC_0.memory.writeWord(0, 0x4420002C, 0x00000000);
    dsDMSC_0.memory.writeWord(0, 0x44200020, 0x80000011);

    dsDMSC_0.memory.writeWord(0, 0x44200034, 0x00080000);
    dsDMSC_0.memory.writeWord(0, 0x44200038, 0x44080000);
    dsDMSC_0.memory.writeWord(0, 0x4420003C, 0x00000000);
    dsDMSC_0.memory.writeWord(0, 0x44200030, 0x80000011);

    dsDMSC_0.memory.writeWord(0, 0x44200044, 0x60000000);
    dsDMSC_0.memory.writeWord(0, 0x44200048, 0x40000000);
    dsDMSC_0.memory.writeWord(0, 0x4420004C, 0x00000000);
    dsDMSC_0.memory.writeWord(0, 0x44200040, 0x8000001D);

    dsDMSC_0.memory.writeWord(0, 0x44200054, 0x80000000);
    dsDMSC_0.memory.writeWord(0, 0x44200058, 0x00000000);
    dsDMSC_0.memory.writeWord(0, 0x4420005C, 0x00000000);
    dsDMSC_0.memory.writeWord(0, 0x44200050, 0x8000001D);

    print("Loading DMSC Firmware ... " + sysfw_bin);
    // Load the DMSC firmware
    dsDMSC_0.memory.loadRaw(0, 0x44000, sysfw_bin, 32, false);
    print("DMSC Firmware Load Done...");
    // Set Stack pointer and Program Counter
    stackPointer = dsDMSC_0.memory.readWord(0, 0x44000);
    progCounter = dsDMSC_0.memory.readWord(0, 0x44004);
    dsDMSC_0.memory.writeRegister("SP", stackPointer);
    dsDMSC_0.memory.writeRegister("PC", progCounter);
    print( "DMSC Firmware run starting now...");
    // Run the DMSC firmware
    dsDMSC_0.target.runAsynch();
    print("Connecting to MCU Cortex_R5_0!");

    // Connect the MCU R5F
    dsMCU1_0.target.connect();

    // This is done to support other boot modes. OSPI is the most stable.
    // MMC is not always stable.
    bootMode = dsMCU1_0.memory.readWord(0, 0x43000030) & 0x78;
    print (" Main Boot Mode is " + bootMode);
    if (bootMode != 0x78)
    {
        print("Disable MCU Timer for ROM clean up");
        dsMCU1_0.memory.writeWord(0, 0x002400010, 0x1); /* Write reset to MCU Timer 0. Left running by ROM */
        dsMCU1_0.memory.writeWord(0, 0x2FFF0430, 0xFFFFFFFF); /* Clear Pending Interrupts */
        dsMCU1_0.memory.writeWord(0, 0x2FFF0018, 0x0); /* Clear Pending Interrupts */
        // Reset the R5F to be in clean state.
        dsMCU1_0.target.reset();
        // Load the board configuration init file.
        dsMCU1_0.expression.evaluate('GEL_Load("'+ ccs_init_elf_file +'")');
        // Run Asynchronously
        dsMCU1_0.target.runAsynch();
        print ("Running Async");
        // Halt the R5F and re-run.
        dsMCU1_0.target.halt();
    }

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
    // Disconnect targets
    dsDMSC_0.target.disconnect();
}

function doEverything()
{
    var run = true;

    if(!File(ccs_init_elf_file).isFile())
    {
        print("[ERROR] File "+ccs_init_elf_file+" not found !!!");
        run = false;
    }

    if(!File(sysfw_bin).isFile())
    {
        print("[ERROR] File "+sysfw_bin+" not found !!!");
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
    sdkPath = System.getenv("MCU_PLUS_SDK_AM64X_PATH");
    if(sdkPath == null)
    {
        // !!! EDIT THIS !!! Add absolute path to SDK in your environment variables
        // OR set this variable to the absolute path of the SDK
        sdkPath = "C:/ti/mcu_plus_sdk";
    }
}

// path to board config elf
ccs_init_elf_file = sdkPath+"/tools/ccs_load/am64x/sciclient_set_boardcfg.release.out";

// path to sysfw bin
sysfw_bin = sdkPath+"/source/drivers/sciclient/soc/am64x_am243x/sysfw.bin"

// !!! EDIT THIS !!! Add absolute path to the CCXML file here.
fileCcxml = "C:/ti/AM64x.ccxml"

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

