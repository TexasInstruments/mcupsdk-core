let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");
let soc = system.getScript(`/drivers/mibspi/soc/mibspi_${common.getSocName()}`);

function getInstanceConfig(moduleInstance) {

    let peripheralPinName = moduleInstance[getPinName(moduleInstance)].$solution.peripheralPinName;
    /* last char is 0 or 1 or 2 i.e xyz_CSn */
    let csNum = peripheralPinName[peripheralPinName.length-1];

    return {
        ...moduleInstance,
        cs: csNum,
    };
};

function pinmuxRequirements(inst) {
    let parent = inst.$ownedBy;
    let interfaceName = inst.interfaceName;
    let config = pinmux.getPinConfigurables(interfaceName,"CS0");
    let instanceName = getPinName(inst);

    if( parent[interfaceName] == undefined)
        return [];

    if((interfaceName == "MSS_MIBSPI") || (interfaceName == "MIBSPI"))
    {
        return [{
            extend: parent[interfaceName],
            name: instanceName,
            displayName: "CS Pin",
            interfaceNames: ["CS0", "CS1", "CS2"],
            config: config,
        }];
    }
    else if(interfaceName == "RCSS_MIBSPI")
    {
        return [{
            extend: parent[interfaceName],
            name: instanceName,
            displayName: "CS Pin",
            interfaceNames: ["CS0", "CS1"],
            config: config,
        }];
    }
}

function getDefaultInterfaceName() {
    return soc.getDefaultInterfaceName();
}

function getPinName(inst) {
     if(inst.interfaceName.includes("RCSS_"))
        return "RCSS_CSn";

    return "MSS_CSn";
}

function getConfigurables()
{
    let config = [];

    config.push(/* Channel attributes */
        {
            name: "peripheralCS",
            displayName: "Peripheral Chip-select",
            default: 0,
            options: [
                {
                    name: 0,
                },
                {
                    name: 1,
                },
                {
                    name: 2,
                },
            ],
            description: "Chipselect in peripheral mode of operation",
        },
        {
            name: "dmaReqLine",
            displayName: "DMA Request Line",
            default: 0,
            options: [
                {
                    name: 0,
                },
                {
                    name: 1,
                },
                {
                    name: 2,
                },
            ],
            description: "DMA reqest line to be used ",
        },
    );
    config.push(
        {
            name: "interfaceName",
            default: getDefaultInterfaceName(),
            hidden: true,
        },
    );

    return config;
}

let mibspi_ch_module_name = "/drivers/mibspi/v0/mibspi_v0_channel";

let mibspi_ch_module = {
    displayName: "MIBSPI Channel Configuration",
    defaultInstanceName: "CONFIG_MIBSPI_CH",
    templates: {
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: mibspi_ch_module_name,
        },
    },
    config: getConfigurables(),
    getInstanceConfig,
    pinmuxRequirements,
    getPinName,
};

exports = mibspi_ch_module;

