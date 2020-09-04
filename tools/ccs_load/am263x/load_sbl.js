/*
 Description: Script used to load SBL executables to R5 Core-0 on the SOC.
 */

////////////Import the DSS packages into our namespace to save on typing//////////
importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);
importPackage(Packages.java.io);
//////////////////////////////////////////////////////////////////////////////////

var debugSession_dap;
var dsCPU = {};

// !!! EDIT THIS !!! Add absolute path to SDK in your environment variables
//                   OR set this variable to the absolute path of the SDK
sdkPath = System.getenv("MCU_PLUS_SDK_AM263X_PATH");
if(sdkPath == null)
{
    sdkPath = "C:/ti/mcu_plus_sdk";
}

// !!! EDIT THIS !!! Edit all of below based on the sbl example you want to run
// sbl_null, sbl_qspi, sbl_uart, sbl_uart_uniflash
runAfterLoad = true;
examplePath = "examples/drivers/boot/sbl_qspi";
exampleName = "sbl_qspi";
os = "nortos";
profile = "release"
board = "am263x-cc";

var elf_file = {};
elf_file["r5fss0-0"] = makeElfFileName("r5fss0-0", os, "ti-arm-clang", exampleName, profile);

/////////////////////////logging date and time////////////////////////////////////
var today = new Date(); 
var time = today.getHours() + "-" + today.getMinutes();
//////////////////////////////////////////////////////////////////////////////////

///////////////////////////Functions to connect////////////////////////////////////
function DAP_connect()
{
	debugSession_dap = debugServer.openSession(".*CS_DAP_0.*");
	debugSession_dap.target.connect();
    debugSession_dap.expression.evaluate('Program_Core_PLL()');
    debugSession_dap.expression.evaluate('Program_SYS_CLK_DIVBY2()');
    debugSession_dap.expression.evaluate('Program_R5F_SYS_CLK_SRC()');
    debugSession_dap.expression.evaluate('GEL_TextOut("*********** R5FSS0 COREA Out of Reset & Unhalted ********\n")');
	print("DAP Connected\n");
}

function DAP_disconnect()
{
	debugSession_dap.target.disconnect();
	wait(10000);
}

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

function wait(ms){
   var start = new Date().getTime();
   var end = start;
   while(end < start + ms) {
     end = new Date().getTime();
  }
  print(ms + " ms waited\n");
}

function runSbl()
{
    /* Set timeout of in units of msecs */
    script.setScriptTimeout(10 * 1000);

    // Open a debug session
    dsCPU["r5fss0-0"] = debugServer.openSession( ".*Cortex_R5_0*" );
    print("Connecting, halting, reseting ...");
    connectHaltResetCpu("r5fss0-0");

    /* Load and Run SBL */
    loadCpu("r5fss0-0");
    runCpu("r5fss0-0");

    print("SBL Execution Done!!!");
}

//////////////////////////////Initialization///////////////////////////////////////
// Create our scripting environment object - which is the main entry point into any script and
// the factory for creating other Scriptable ervers and Sessions
script = ScriptingEnvironment.instance();

// Get the Debug Server and start a Debug Session
debugServer = script.getServer("DebugServer.1");

///////////////////////////////Connecting DAP/////////////////////////////////////
DAP_connect();
runSbl();
