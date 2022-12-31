
let common = system.getScript("/common");
let soc = system.getScript(`/industrial_comms/ethercat/ethercat_${common.getSocName()}`);
let device = common.getDeviceName();
let is_mdio_workaround_device = ((device === "am64x-evm") || (device === "am243x-evm") || (device === "am243x-lp")) ? true : false;

function pinmuxRequirements(inst) {
    return soc.getPinmuxRequirements(inst);
}

function getInterfaceNameList(inst) {

    return soc.getInterfaceNameList(inst);
}

function getPeripheralPinNames(inst)
{
    return soc.getPeripheralPinNames(inst);
}

function getConfigurables()
{
    /* get 'CPU enable' configurables */
    let config = [];
    let icssConfig = soc.getIcssInstancesArr();

    config.push(common.ui.makeConfig(icssConfig, "instance", "ICSS Instance"));

    if(is_mdio_workaround_device) {
        config.push({
            name: "manualMode",
            description: "Enable MDIO Manual Mode",
            displayName: "Enable MDIO Manual Mode",
            default: true,
            onChange: (inst, ui) => {
                if(inst.manualMode)
                    ui.mdioManualModeBaseAddr.hidden = false;
                else
                    ui.mdioManualModeBaseAddr.hidden = true;
            },
        });
        config.push({
            name: "mdioManualModeBaseAddr",
            displayName: "MDIO Manual Mode Base Address",
            default: 0x00010E40,
            readOnly: true,
            displayFormat: "hex",
            onChange: (inst, ui) => {
                if(inst.manualMode)
                    ui.mdioManualModeBaseAddr.hidden = false;
            },
        });
    }

    return config;
}

let ethercat_module_name = "/industrial_comms/ethercat/ethercat";

let ethercat_module = {

    displayName: "EtherCAT",
    templates: {
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: ethercat_module_name,
        },
    },
    defaultInstanceName: "CONFIG_ETHERCAT",
    config: getConfigurables(),
    moduleStatic: {
        modules: function(inst) {
            return [{
                name: "system_common",
                moduleName: "/system_common",
            }]
        },
    },
    pinmuxRequirements,
    getInterfaceNameList,
    getPeripheralPinNames,
    sharedModuleInstances: sharedModuleInstances,
    moduleInstances: moduleInstances,
};

function sharedModuleInstances(instance) {
    let modInstances = new Array();
    let icssRequiredArgs = soc.getRequiredArgsIcssInstance(instance);

    modInstances.push({
        name: "icss",
        displayName: "PRU Configuration",
        moduleName: '/drivers/pruicss/pruicss',
        requiredArgs: icssRequiredArgs,
    });

    return (modInstances);
}

function moduleInstances(instance) {
    let modInstances = new Array();

    modInstances.push({
        name: "ethphy",
        displayName: "ETHPHY Configuration",
        moduleName: '/board/ethphy/ethphy',
        useArray: true,
        maxInstanceCount: 2,
        minInstanceCount: 2,
        defaultInstanceCount: 2,
        requiredArgs: (() => {
            if(is_mdio_workaround_device) {
                return  {
                            mdioInstance: instance.instance,
                            manualMode: instance.manualMode,
                            mdioManualModeBaseAddr: instance.mdioManualModeBaseAddr,
                        };
            } else {
                return  {
                            mdioInstance: instance.instance,
                        };
            }
        })(),
    });

    return (modInstances);
}

exports = ethercat_module;
