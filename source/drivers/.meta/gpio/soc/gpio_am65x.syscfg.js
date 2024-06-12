let common = system.getScript("/common");

function getInterfaceName(inst) {

    if(inst.useMcuDomainPeripherals)
        return "MCU_GPIO"

    if(inst.useWakeUpDomainPeripherals)
        return "WKUP_GPIO"

    return "GPIO";
}

function getMaxInterruptRouters(inst) {

    let mcuRouterOutput = 16;
    let mainRouterOutput = 32;
    if(common.getSelfSysCfgCoreName().includes("r5f"))
        return mcuRouterOutput
    return mainRouterOutput;
}

function getInstanceString(moduleInstance) {
    let interfaceName = getInterfaceName(moduleInstance);
    let solution = moduleInstance[interfaceName].$solution

    return solution.peripheralName;
}

function getPinIndex(moduleInstance) {
    let interfaceName = getInterfaceName(moduleInstance);
    let peripheralPinName = moduleInstance[interfaceName].gpioPin.$solution.peripheralPinName;

    if(! peripheralPinName)
        return "INVALID";

    let splitStrings = peripheralPinName.split("_");

    /* The last split is the GPIO number */

    return splitStrings[splitStrings.length-1];
}

function getCpuRouterId() {
    let router_map = {
        "r5fss0-0" : "MCU0_INTR",
        "r5fss0-1" : "MCU0_INTR",
    };

    return router_map[common.getSelfSysCfgCoreName()];
}

function getTisciDestCoreID() {
    let corename_map = {
        "r5fss0-0" : "MCU_ARMSS0_CPU0",
        "r5fss0-1" : "MCU_ARMSS0_CPU0",
    };

    return corename_map[common.getSelfSysCfgCoreName()];
}

exports = {
    getInterfaceName,
    getInstanceString,
    getPinIndex,
    getMaxInterruptRouters,
    getCpuRouterId,
    getTisciDestCoreID,
};
