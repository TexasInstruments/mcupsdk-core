let common = system.getScript("/common");

function getInterfaceName(inst) {

    if(inst.useMcuDomainPeripherals)
        return "MCU_GPIO"

    return "GPIO";
}

function getMaxInterruptRouters(inst) {

    let mcuRouterOutput = 11;
    let mainRouterOutput = 53;
    if(common.getSelfSysCfgCoreName().includes("m4f"))
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

exports = {
    getInterfaceName,
    getInstanceString,
    getPinIndex,
    getMaxInterruptRouters,
};
