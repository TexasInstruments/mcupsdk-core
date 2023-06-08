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

function getCpuRouterId() {
    let router_map = {
        "m4fss0-0" : "MCU_M4FSS0_CORE0_NVIC_MCU_MCU",
        "r5fss0-0" : "R5FSS0_CORE0_INTR_MAIN",
        "r5fss0-1" : "R5FSS0_CORE1_INTR_MAIN",
        "r5fss1-0" : "R5FSS1_CORE0_INTR_MAIN",
        "r5fss1-1" : "R5FSS0_CORE0_INTR_MAIN",
        "a53ss0-0" : "GICSS0_SPI_MAIN",
        "a53ss0-1" : "GICSS0_SPI_MAIN",
    };

    return router_map[common.getSelfSysCfgCoreName()];
}

function getTisciDestCoreID() {
    let corename_map = {
        "m4fss0-0" : "MCU_M4FSS0_CORE0",
        "r5fss0-0" : "R5FSS0_CORE0",
        "r5fss0-1" : "R5FSS0_CORE1",
        "r5fss1-0" : "R5FSS1_CORE0",
        "r5fss1-1" : "R5FSS1_CORE1",
        "a53ss0-0" : "GICSS0",
        "a53ss0-1" : "GICSS0",
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
