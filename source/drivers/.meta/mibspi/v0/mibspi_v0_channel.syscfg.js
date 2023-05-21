let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");
let soc = system.getScript(`/drivers/mibspi/soc/mibspi_${common.getSocName()}`);

function getInstanceConfig(moduleInstance) {

    let peripheralPinName = null;
    if (moduleInstance[getPinName(moduleInstance)] != null)
        peripheralPinName = moduleInstance[getPinName(moduleInstance)].$solution.peripheralPinName;
    let csNum;
    if (peripheralPinName != null) {
        /* last char is 0 or 1 or 2 or 3 i.e xyz_CSn */
        csNum = peripheralPinName[peripheralPinName.length-1];
    } else {
        /* If CS pin is unchecked/in 3 pin mode, use channel 0 for reigster access. */
        csNum = 0;
    }

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
    if(inst.pinMode == "3PIN")
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
    if(interfaceName == "RCSS_MIBSPI")
    {
        return [{
            extend: parent[interfaceName],
            name: instanceName,
            displayName: "CS Pin",
            interfaceNames: ["CS0", "CS1"],
            config: config,
        }];
    }
    return [];
}

function getDefaultInterfaceName() {
    return soc.getDefaultInterfaceName();
}

function getPinName(inst) {
    if(inst.interfaceName.includes("RCSS_"))
        return "RCSS_CSn";

    if(inst.pinMode == "3PIN")
        return "";

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
        {
            name: "pinMode",
            displayName: "Pin Mode",
            default: "4PIN_CS",
            options: [
                {
                    name: "3PIN",
                    displayName: "3 Pin Mode"
                },
                {
                    name: "4PIN_CS",
                    displayName: "4 Pin Mode with CS"
                },
            ],
            hidden: true,
            description: "3 pin mode: Chip-select (CS) is not used and all related options to CS have no meaning. 4 pin mode: CS is used.",
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

