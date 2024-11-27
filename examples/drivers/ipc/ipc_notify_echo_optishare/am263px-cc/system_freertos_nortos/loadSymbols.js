importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);
importPackage(java.io);
importPackage(java.lang);

// EDIT THIS: 
var sdk_path = ""
var ccxmlPath = "AM263px.ccxml";
var rfss00_core = sdk_path + "mcu_plus_sdk/examples/drivers/ipc/ipc_notify_echo_optishare/am263px-cc/r5fss0-0_freertos/ti-arm-clang/ipc_notify_echo_optishare.release.optishare.out";
var rfss01_core = sdk_path + "mcu_plus_sdk/examples/drivers/ipc/ipc_notify_echo_optishare/am263px-cc/r5fss0-1_nortos/ti-arm-clang/ipc_notify_echo_optishare.release.optishare.out";
var rfss10_core = sdk_path + "mcu_plus_sdk/examples/drivers/ipc/ipc_notify_echo_optishare/am263px-cc/r5fss1-0_nortos/ti-arm-clang/ipc_notify_echo_optishare.release.optishare.out";
var rfss11_core = sdk_path + "mcu_plus_sdk/examples/drivers/ipc/ipc_notify_echo_optishare/am263px-cc/r5fss1-1_nortos/ti-arm-clang/ipc_notify_echo_optishare.release.optishare.out";
var sso = sdk_path + "mcu_plus_sdk/examples/drivers/ipc/ipc_notify_echo_optishare/am263px-cc/system_freertos_nortos/sso.out";

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

    debugSession_r5fss00.symbol.unloadAllSymbols();
    debugSession_r5fss01.symbol.unloadAllSymbols();
    debugSession_r5fss10.symbol.unloadAllSymbols();
    debugSession_r5fss11.symbol.unloadAllSymbols();

    // Load the program
    // first load sso then load other programs
    debugSession_r5fss00.symbol.add(rfss00_core);
    debugSession_r5fss01.symbol.add(rfss01_core);
    debugSession_r5fss10.symbol.add(rfss10_core);
    debugSession_r5fss11.symbol.add(rfss11_core);

    //load symbols of sso in every core
    debugSession_r5fss00.symbol.add(sso);
    debugSession_r5fss01.symbol.add(sso);
    debugSession_r5fss10.symbol.add(sso);
    debugSession_r5fss11.symbol.add(sso);
}

loadRunAll();
