

let common = system.getScript("/common");
let soc = system.getScript(`/board/ethphy/ethphy_${common.getSocName()}`);
let pruicss = system.getScript(`/drivers/pruicss/pruicss`);
let device = common.getDeviceName();
let is_mdio_workaround_device = ((device === "am64x-evm") || (device === "am243x-evm") || (device === "am243x-lp")) ? true : false;

function getInstanceConfig(moduleInstance) {
    let configArr = soc.getConfigArr();
    let config = configArr.find(o => o.name === moduleInstance.name);

    config.mdioBaseAddr = pruicss.getMdioBaseAddr(moduleInstance.mdioInstance);

    return {
        ...config,
        ...moduleInstance,
    };
};

function getNumValidInstances(module) {
    let num_instances = 0;

    for(let i = 0; i < module.$instances.length; i++) {
        let instance = module.$instances[i];
        let config = module.getInstanceConfig(instance);
        if(config.name != "NONE")
        {
            num_instances = num_instances + 1;
        }
    }
    return num_instances;
}

function deviceSelectionOnChange (inst, ui) {

    if(inst.name == "NONE") {
        ui.mdioInstance.hidden = true;
        ui.mdioPort.hidden = true;
        ui.customDeviceName.hidden = true;
    }
    else
    if(inst.name == "CUSTOM") {
        ui.mdioInstance.hidden = false;
        ui.mdioPort.hidden = false;
        ui.customDeviceName.hidden = false;
    }
    else
    {
        ui.mdioInstance.hidden = false;
        ui.mdioPort.hidden = false;
        ui.customDeviceName.hidden = true;
    }
}

function getConfigurables()
{
    /* get 'CPU enable' configurables */
    let config = [];
    let staticConfig = soc.getConfigArr();
    let mdioConfig = soc.getMdioInstancesArr();

    let deviceSelectionConfig = common.ui.makeConfig(staticConfig, "name", "ETHPHY Device");

    deviceSelectionConfig.onChange = deviceSelectionOnChange;

    config.push( deviceSelectionConfig);

    config.push(
        {
            name: "customDeviceName",
            description: "Name of the custom ETHPHY device as implemented using the ETHPHY driver interface",
            displayName: "Custom Device Name",
            default: "MyEthPhyDevice",
            hidden: true,

        },
        common.ui.makeConfig(mdioConfig, "mdioInstance", "MDIO Instance"),
        {
            name: "mdioPort",
            description: "MDIO Phy Address to use. Value MUST be between 0 .. 31",
            displayName: "MDIO Phy Address",
            default: 15,
        },
    );

    if(is_mdio_workaround_device) {
        config.push(
            {
                name: "manualMode",
                description: "Enable MDIO Manual Mode",
                displayName: "Enable MDIO Manual Mode",
                default: false,
                hidden: true,
            },
            {
                name: "mdioManualModeBaseAddr",
                displayName: "MDIO Manual Mode Base Address",
                default: 0x30090E40,
                displayFormat: "hex",
                hidden: true,
            },
            {
                name: "mdioManualModeLinkPolling",
                displayName: "MDIO Manual Mode Link Status Update",
                default: "Polling",
                options: [
                    {
                        name: "MLINK",
                        displayName: "MLINK Based",
                        description: "In this MLINK pins for getting link status updates from the PHY",
                    },
                    {
                        name: "Polling",
                        displayName: "PHY Polling Based",
                        description: "In this MDIO workaround FW Polls the PHY register for link status",
                    }
                ],
                hidden: true,
            },
        );
    }

    return config;
}

let ethphy_module_name = "/board/ethphy/ethphy";

let ethphy_module = {
    displayName: "ETHPHY",

    templates: {
        "/board/board/board_open_close.c.xdt": {
            board_open_close_config: "/board/ethphy/templates/ethphy_open_close_config.c.xdt",
            board_open: "/board/ethphy/templates/ethphy_open.c.xdt",
            board_close: "/board/ethphy/templates/ethphy_close.c.xdt",
        },
        "/board/board/board_open_close.h.xdt": {
            board_open_close_config: "/board/ethphy/templates/ethphy_open_close.h.xdt",
        },
        "/board/board/board_config.h.xdt": {
            board_config: "/board/ethphy/templates/ethphy.h.xdt",
        },

    },
    defaultInstanceName: "CONFIG_ETHPHY",
    config: getConfigurables(),
    validate: validate,
    moduleStatic: {
        modules: function(inst) {
            return [{
                name: "system_common",
                moduleName: "/system_common",
            }]
        },
    },
    getInstanceConfig,
    getNumValidInstances,
};

function validate(inst, report) {
    common.validate.checkNumberRange(inst, report, "mdioPort", 0, 31, "dec");
}

exports = ethphy_module;
