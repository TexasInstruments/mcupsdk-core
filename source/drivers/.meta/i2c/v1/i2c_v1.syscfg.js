
let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");
let soc = system.getScript(`/drivers/i2c/soc/i2c_${common.getSocName()}`);

function getStaticConfigArr() {
    return system.getScript(`/drivers/i2c/soc/i2c_${common.getSocName()}`).getStaticConfigArr();
}

function getInstanceConfig(moduleInstance) {
    let solution = moduleInstance[getInterfaceName(moduleInstance)].$solution;
    let staticConfigArr = getStaticConfigArr();
    let staticConfig = staticConfigArr.find( o => o.name === solution.peripheralName);

    return {
        ...staticConfig,
        ...moduleInstance
    }
};

function pinmuxRequirements(inst) {
    let interfaceName = getInterfaceName(inst);

    let resources = [];

    resources.push( pinmux.getPinRequirements(interfaceName, "SCL", "I2C Clock Pin"));
    resources.push( pinmux.getPinRequirements(interfaceName, "SDA", "I2C Data Pin"));

    let peripheral = {
        name          : interfaceName,
        displayName   : "I2C Instance",
        interfaceName : interfaceName,
        resources     : resources,
        signalTypes   : {
            sclPin     : ['SCL'],
            sdaPin     : ['SDA'],
        }
    };

    return [peripheral];
}

function getInterfaceName(inst) {

    return soc.getInterfaceName(inst);
}

function getPeripheralPinNames(inst) {

    return [ "SCL", "SDA" ];
}

function getClockEnableIds(inst) {

    let instConfig = getInstanceConfig(inst);

    return instConfig.clockIds;
}

function getConfigurables()
{
    let config = [];

    config.push(

        {
            name: "bitRate",
            displayName: "Bit Rate",
            default: "400KHZ",
            description: "I2C bitrate",
            options: [
                {
                    name: "100KHZ",
                    displayName: "100 KHZ"
                },
                {
                    name: "400KHZ",
                    displayName: "400 KHZ"
                },
            ],
        },
        {
            name: "enableIntr",
            displayName: "Enable Interrupt",
            default: true,
            hidden: false,
            onChange: function (inst, ui) {
                let hideConfigs = false;
                if(inst.enableIntr == false) {
                    hideConfigs = true;
                    inst.transferCallbackFxn = "NULL";
                    inst.transferMode = "BLOCKING";
                    ui.transferCallbackFxn.hidden = true;
                }
                ui.transferMode.hidden = hideConfigs;
            },
            description: "If enabled interrupt mode otherwise polling mode",
        },
        {
            name: "transferMode",
            displayName: "Transfer Mode",
            default: "BLOCKING",
            hidden: false,
            options: [
                {
                    name: "BLOCKING",
                    displayName: "Blocking"
                },
                {
                    name: "CALLBACK",
                    displayName: "Callback"
                },
            ],
            onChange: function (inst, ui) {
                if(inst.transferMode == "CALLBACK") {
                    ui.transferCallbackFxn.hidden = false;
                }
                else{
                    inst.transferCallbackFxn = "NULL";
                    ui.transferCallbackFxn.hidden = true;
                }
            },
            description: "This determines whether the driver operates synchronously or asynchronously",
        },
        {
            name: "transferCallbackFxn",
            displayName: "Transfer Callback",
            default: "NULL",
            hidden: true,
            description: "Transfer callback function when callback mode is selected",
        },
        {
            name: "advanced",
            displayName: "Show Advanced Config",
            default: false,
            onChange: function (inst, ui) {
                let hideConfigs = true;
                if(inst.advanced == true) {
                    hideConfigs = false;
                }
                ui.ownTargetAddr.hidden = hideConfigs;
            },
        },
        {
            name: "ownTargetAddr",
            displayName: "Own Target Address (0x00 - 0x7F)",
            default: 0x1C,
            hidden: true,
            displayFormat: "hex"
        },
        {
            name: "sdkInfra",
            displayName: "SDK Infra",
            default: "HLD",
            options: [
                {
                    name: "HLD",
                    displayName: "HLD"
                },
                {
                    name: "LLD",
                    displayName: "LLD"
                },
            ],
            onChange: function (inst, ui) {
                if(inst.sdkInfra == "LLD") {
                    inst.transferMode = "BLOCKING";
                    inst.transferCallbackFxn = "NULL";
                    ui.transferCallbackFxn.hidden = true;
                    ui.transferMode.hidden = true;
                    inst.enableIntr = false;
                    ui.enableIntr.hidden = true;
                    if(inst.enableIntr == "NULL") {
                        /* Clear NULL entry as user need to provide a fxn */
                        inst.enableIntr = false;
                    }
                }
                else {
                    ui.enableIntr.hidden = false;
                    inst.enableIntr = false;
                }
            },
            description: "SDK Infra",
        },
    )

    if(soc.isMakeInstanceRequired())
    {
        config.push(common.ui.makeInstanceConfig(getStaticConfigArr()));
    }

    return config;

}

let i2c_module_name = "/drivers/i2c/i2c";

let i2c_module = {
    displayName: "I2C",
    templates: {
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: i2c_module_name,
        },
        "/drivers/system/power_clock_config.c.xdt": {
            moduleName: i2c_module_name,
        },
    },
    moduleInstances: moduleInstances,
    defaultInstanceName: "CONFIG_I2C",
    config:  getConfigurables(),
    validate : validate,
    moduleStatic: {
        modules: function(inst) {
            return [{
                name: "system_common",
                moduleName: "/system_common",
            }]
        },
    },
    pinmuxRequirements,
    validatePinmux: validatePinmux,
    getInstanceConfig,
    getInterfaceName,
    getPeripheralPinNames,
    getClockEnableIds,
};

function validate(instance, report) {
    common.validate.checkNumberRange(instance, report, "ownTargetAddr", 0x0, 0x7F, "hex");
    common.validate.checkValidCName(instance, report, "transferCallbackFxn");
    if((instance.transferMode == "CALLBACK") &&
        ((instance.transferCallbackFxn == "NULL") ||
            (instance.transferCallbackFxn == ""))) {
        report.logError("Callback function MUST be provided for callback transfer mode", instance, "transferCallbackFxn");
    }
}

/*
 *  ======== validatePinmux ========
 */
function validatePinmux(instance, report) {
    let instConfig = getInstanceConfig(instance);
    if((instance.enableIntr == true) &&
        (instConfig.name == "MSS_I2CA") &&
            (common.getSelfSysCfgCoreName().includes("c66"))){
        report.logError("Interrupt mode is not supported for this instance", instance, "enableIntr");
    }
}

/*
 *  ======== moduleInstances ========
 */
function moduleInstances(inst) {
    let modInstances = new Array();

    if( inst.sdkInfra == "HLD")
    {
        modInstances.push({
            name: "I2C_child",
            moduleName: '/drivers/i2c/v1/i2c_v1_template',
            },
        );
    }
    else
    {
        modInstances.push({
            name: "I2C_child",
            moduleName: '/drivers/i2c/v1/i2c_v1_template_lld',
            },
        );
    }

    return (modInstances);
}

function getClockFrequencies(inst) {

    let instConfig = getInstanceConfig(inst);

    return instConfig.clockFrequencies;
}

function getModule()
{
    let module = i2c_module;
    if(soc.isFrequencyDefined())
    {
        module.getClockFrequencies = getClockFrequencies;
    }
    return module;
}

exports = getModule();
