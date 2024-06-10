/*
 * Copyright (c) 2024 Texas Instruments Incorporated
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

2. Launch AM65x target connection in CCS, however DO NOT connect to any CPUs.

3. From CCS Scripting console do (CCS Tool Bar > View > Scripting Console)
    js:> loadJSFile "<path/to/sdk>/tools/ccs_load/am65x/load_dmsc.js"

4. After successful execution you should see a log like below

  On the CCS scripting console in CCS,

    js:> loadJSFile "C:/ti/mcu_plus_sdk/tools/ccs_load/am65x/load_dmsc.js"
    Connecting to DMSC_Cortex_M3_0!
	Loading DMSC Firmware...
	DMSC Firmware Load Done...
	DMSC Firmware run starting now...
	Connecting to MCU Cortex_R5_0!
	Running the board configuration initialization from R5!
	Initializing DDR!
	Happy Debugging!!

    js:>

  On the AM65x.ccxml:CIO console in CCS, i.e the R5F console,

    [MCU_PULSAR_Cortex_R5_0]
	DMSC Firmware Version 22.1.1--v2022.01 (Terrific Llam
	DMSC Firmware revision 0x16
	DMSC ABI revision 3.1

	[SCICLIENT] ABI check PASSED
	[SCICLIENT] Board Configuration with Debug enabled ...
	[SCICLIENT] Common Board Configuration PASSED
	[SCICLIENT] PM Board Configuration PASSED
	[SCICLIENT] RM Board Configuration PASSED
	[SCICLIENT] Security Board Configuration PASSED

	DMSC Firmware Version 22.1.1--v2022.01 (Terrific Llam
	DMSC Firmware revision 0x16
	DMSC ABI revision 3.1

	All tests have passed!!

  On the AM65x.ccxml console in CCS, i.e the GEL output console,

    DMSC_Cortex_M3_0: GEL Output: Detected Silicon Rev 2.0
	DMSC_Cortex_M3_0: GEL Output: Configuring AM65xEVM...
	DMSC_Cortex_M3_0: GEL Output: Init value actual value: 0x00000888
	DMSC_Cortex_M3_0: GEL Output: Register value: 0x00000888
	DMSC_Cortex_M3_0: GEL Output: ATCM is on
	DMSC_Cortex_M3_0: GEL Output: ATCM configured.
    ...
	DMSC_Cortex_M3_0: GEL Output: Powering up all PSC power domains in progress...
	DMSC_Cortex_M3_0: GEL Output: Powering up LPSC_WKUP_COMMON
    ...
	DMSC_Cortex_M3_0: GEL Output: Powering up LPSC_A53_CLUSTER_0
    ...
	DMSC_Cortex_M3_0: GEL Output: Power domain and module state changed successfully.
	DMSC_Cortex_M3_0: GEL Output: Powering up all PSC power domains done!
	MCU_PULSAR_Cortex_R5_0: GEL Output:

	DDR not initialized with R5 connect.

	Go to menu Scripts --> DDR_Initialization to initialize DDR.

	====

5. If any of the logs in step 4 show "fail" or "error" messages then
   check your EVM, CCS, SDK setup and try again.
*/

function updateScriptVars()
{
    //Open a debug session
    dsMCU1_0 = debugServer.openSession( ".*MCU_PULSAR_Cortex_R5_0" );
    dsDMSC_0 = debugServer.openSession( ".*DMSC_Cortex_M3_0" );
}

function printVars()
{
    updateScriptVars();
}

function connectTargets()
{
    /* Set timeout of 20 seconds */
    script.setScriptTimeout(60000);
    updateScriptVars();
    sysResetVar=dsDMSC_0.target.getResetType(1);
    sysResetVar.issueReset();
    print("Connecting to DMSC_Cortex_M3_0!");

    // Connect targets
    dsDMSC_0.target.connect();
    dsMCU1_0.target.connect();

    // Read the Device ID to load the correct SYSFW
    dev_id = dsMCU1_0.memory.readWord(0, 0x43000014); // CSL_WKUP_CTRL_MMR0_CFG0_BASE + CSL_WKUP_CTRL_MMR_CFG0_JTAGID

    print("Loading DMSC Firmware...");
    // Load the DMSC firmware
    if (dev_id == 0x1BB5A02F) {
        dsDMSC_0.memory.loadRaw(0, 0x40000, sysfw_bin, 32, false);
    } else {
        print("Invalid Device ID!");
        return;
    }

    print("DMSC Firmware Load Done...");
    // Set Stack pointer and Program Counter
    stackPointer = dsDMSC_0.memory.readWord(0, 0x40000);
    progCounter = dsDMSC_0.memory.readWord(0, 0x40004);
    dsDMSC_0.memory.writeRegister("SP", stackPointer);
    dsDMSC_0.memory.writeRegister("PC", progCounter);
    print( "DMSC Firmware run starting now...");
    // Run the DMSC firmware
    dsDMSC_0.target.runAsynch();
    print("Connecting to MCU Cortex_R5_0!");

    // Connect the MCU R5F
    dsMCU1_0.target.connect();

    print("Running the board configuration initialization from R5!");
    // Load the board configuration init file.
    dsMCU1_0.memory.loadProgram(ccs_init_elf_file);
    // Halt the R5F and re-run.
    dsMCU1_0.target.halt();
    dsMCU1_0.target.reset();
    dsMCU1_0.target.restart();
    // Run Synchronously for the executable to finish
    dsMCU1_0.target.run();

	return 0;
}

function disconnectTargets()
{
    updateScriptVars();

    // Disconnect targets
    dsDMSC_0.target.disconnect();
    // Reset the R5F to be in clean state.
    dsMCU1_0.target.reset();
    print("Initializing DDR!");
    // Execute DDR initialization script from R5F.
    try
    {
        dsMCU1_0.expression.evaluate("DDR4_800MHz_Initialization_for_EVM()");
        dsMCU1_0.expression.evaluate("timer0_cleanup()");
    }
    catch(e)
    {
        print("Some error in GEL execution for DDR4_CONFIG");
    }
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
    sdkPath = System.getenv("MCU_PLUS_SDK_AM65X_PATH");
    if(sdkPath == null)
    {
        // !!! EDIT THIS !!! Add absolute path to SDK in your environment variables
        // OR set this variable to the absolute path of the SDK
        sdkPath = "C:/ti/mcu_plus_sdk";
    }
}

// path to board config elf
ccs_init_elf_file = sdkPath+"/tools/ccs_load/am65x/sciclient_set_boardcfg.release.out";

// path to sysfw bin
sysfw_bin = sdkPath+"/source/drivers/sciclient/soc/am65x/ti-sci-firmware-am65x_sr2-gp.bin"

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
        print("Seems like the script is not run from within CCS. Please edit the load_dmsc.js script to add a path to your CCXML configuration file in this case.")
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

