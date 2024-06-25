let common = system.getScript("/common");

function getInterfaceName(inst) {

    return "GPIO";
}

function getInstanceString(moduleInstance) {

    let cpu = common.getSelfSysCfgCoreName();
    let instanceString = "";

    switch(cpu) {
        case "r5fss0-0":
            instanceString = "GPIO0_U";
            break;
        case "r5fss0-1":
            instanceString = "GPIO1_U";
            break;
        case "r5fss1-0":
            instanceString = "GPIO2_U";
            break;
        case "r5fss1-1":
            instanceString = "GPIO3_U";
            break;
    }

    return instanceString;
}

function getPinIndex(moduleInstance) {
    let interfaceName = getInterfaceName(moduleInstance);
    let peripheralPinName = moduleInstance["GPIO_n"].$solution.peripheralPinName;

    if(! peripheralPinName)
        return "INVALID";

    let pinIndex = peripheralPinName.replace("GPIO","");

    /* The last part is the GPIO pin index */

    return pinIndex;
}

exports = {
    getInterfaceName,
    getInstanceString,
    getPinIndex,
};
