importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);
importPackage(java.io);
importPackage(java.lang);

var sdkPath = "";
var ccxmlPath = "AM263px.ccxml";
var rfss00_core = sdkPath + "mcu_plus_sdk/examples/drivers/ipc/ipc_notify_echo_optishare/am263px-cc/r5fss0-0_freertos/ti-arm-clang/ipc_notify_echo_optishare.debug.optishare.out";
var rfss01_core = sdkPath + "mcu_plus_sdk/examples/drivers/ipc/ipc_notify_echo_optishare/am263px-cc/r5fss0-1_nortos/ti-arm-clang/ipc_notify_echo_optishare.debug.optishare.out";
var rfss10_core = sdkPath + "mcu_plus_sdk/examples/drivers/ipc/ipc_notify_echo_optishare/am263px-cc/r5fss1-0_nortos/ti-arm-clang/ipc_notify_echo_optishare.debug.optishare.out";
var rfss11_core = sdkPath + "mcu_plus_sdk/examples/drivers/ipc/ipc_notify_echo_optishare/am263px-cc/r5fss1-1_nortos/ti-arm-clang/ipc_notify_echo_optishare.debug.optishare.out";
var sso = sdkPath + "mcu_plus_sdk/examples/drivers/ipc/ipc_notify_echo_optishare/am263px-cc/system_freertos_nortos/sso.out";

function loadRunAll()
{
    // Open a debug session
    var debugSession_r5fss00 = ds.openSession(".*Cortex_R5_0");
    var debugSession_r5fss01 = ds.openSession(".*Cortex_R5_1");
    var debugSession_r5fss10 = ds.openSession(".*Cortex_R5_2");
    var debugSession_r5fss11 = ds.openSession(".*Cortex_R5_3");

    debugSession_r5fss00.target.disconnect();
    debugSession_r5fss01.target.disconnect();
    debugSession_r5fss10.target.disconnect();
    debugSession_r5fss11.target.disconnect();

    // Connect to the target
    debugSession_r5fss00.target.connect();
    debugSession_r5fss01.target.connect();
    debugSession_r5fss10.target.connect();
    debugSession_r5fss11.target.connect();

    //reset the target
    debugSession_r5fss00.target.reset();
    debugSession_r5fss01.target.reset();
    debugSession_r5fss10.target.reset();
    debugSession_r5fss11.target.reset();

    debugSession_r5fss00.symbol.unloadAllSymbols();
    debugSession_r5fss01.symbol.unloadAllSymbols();
    debugSession_r5fss10.symbol.unloadAllSymbols();
    debugSession_r5fss11.symbol.unloadAllSymbols();

    // Load the program
    // first load sso then load other programs
    debugSession_r5fss00.memory.loadProgram(sso);
    debugSession_r5fss00.memory.loadProgram(rfss00_core);
    debugSession_r5fss01.memory.loadProgram(rfss01_core);
    debugSession_r5fss10.memory.loadProgram(rfss10_core);
    debugSession_r5fss11.memory.loadProgram(rfss11_core);

    //load symbols of sso in every core
    debugSession_r5fss00.symbol.add(sso);
    debugSession_r5fss01.symbol.add(sso);
    debugSession_r5fss10.symbol.add(sso);
    debugSession_r5fss11.symbol.add(sso);

    debugSession_r5fss00.breakpoint.removeAll();
    debugSession_r5fss01.breakpoint.removeAll();
    debugSession_r5fss10.breakpoint.removeAll();
    debugSession_r5fss11.breakpoint.removeAll();


    debugSession_r5fss00.breakpoint.add("0x4");
    debugSession_r5fss00.breakpoint.add("0xc");
    debugSession_r5fss00.breakpoint.add("0x10");

    debugSession_r5fss01.breakpoint.add("0x4");
    debugSession_r5fss01.breakpoint.add("0xc");
    debugSession_r5fss01.breakpoint.add("0x10");

    debugSession_r5fss10.breakpoint.add("0x4");
    debugSession_r5fss10.breakpoint.add("0xc");
    debugSession_r5fss10.breakpoint.add("0x10");

    debugSession_r5fss11.breakpoint.add("0x4");
    debugSession_r5fss11.breakpoint.add("0xc");
    debugSession_r5fss11.breakpoint.add("0x10");
    
}

loadRunAll();
