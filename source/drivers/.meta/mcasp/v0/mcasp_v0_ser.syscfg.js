
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/mcasp/soc/mcasp_${common.getSocName()}`);
let pinmux = system.getScript("/drivers/pinmux/pinmux");

function getConfigArr() {
    return soc.getConfigArr();
}

function getInstanceConfig(moduleInstance) {
    let solution = moduleInstance[getPinName(moduleInstance)].$solution;
    let configArr = getConfigArr();
    let config = configArr.find( o => o.name === solution.peripheralName);

     return {
        ...config,
        ...moduleInstance,
     };
};

function getParentInstanceConfig(moduleInstance) {
    let configArr = getConfigArr();
    let config = configArr.find( o => o.name === moduleInstance.instance);
    return {
        ...config,
        ...moduleInstance,
     };
};

function getPinName(inst) {
    const serializerNum = _.clamp(inst.serNum, 0, 16)
       return `DAT${serializerNum}`;
}

function pinmuxRequirements(inst) {
    let parent = inst.$ownedBy;
    let interfaceName = inst.interfaceName;
    let config = pinmux.getPinConfigurables(interfaceName,"DAT0");

    if( parent[interfaceName] == undefined) {
        return [];
    }
    const serializerNum = _.clamp(inst.serNum, 0, 16)
    return [{
        extend: parent[interfaceName],
        name: `DAT${serializerNum}`,
        displayName: `DAT${serializerNum} Pin`,
        interfaceNames: [`DAT${serializerNum}`],
        config: config,
    }];
}

let mcasp_ser_module_name = "/drivers/mcasp/v0/mcasp_v0_ser";

let mcasp_ser_module = {
    displayName: "MCASP Serializer Configuration",
    defaultInstanceName: "CONFIG_MCASP_SER",
    templates: {
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: mcasp_ser_module_name,
        },
    },
    config: [
        /*  serializer number */
        {
            name: "serNum",
            displayName: "Serializer Number",
            default: 0,
            description: "Serializer Number",
            displayFormat: "dec",
        },
        /*  Transmit/Receive Config */
        {
            name: "dataDir",
            displayName: "Data Direction",
            description: "Data Direction Transmit/Receive",
            default: "Transmit",
            options: [
                { name: "Transmit", displayName: "Transmit"},
                { name: "Receive", displayName: "Receive"},
            ],
        },
        {
            name: "interfaceName",
            default: "RCSS_MCASP",
            hidden: true,
        },
        {
            name: "enableLoopback",
            default: true,
            hidden: true,
        },
    ],
    validate : validate,
    pinmuxRequirements,
    getInstanceConfig,
    getParentInstanceConfig,
    getPinName,
};

/*
 *  ======== validate ========
 */
function validate(inst, report) {
    let parent = inst.$ownedBy;
    let parentInstConfig = getParentInstanceConfig(parent);
    common.validate.checkNumberRange(inst, report, "serNum", 0, Number(parentInstConfig.numSerializers -1), "dec");
}

exports = mcasp_ser_module;
