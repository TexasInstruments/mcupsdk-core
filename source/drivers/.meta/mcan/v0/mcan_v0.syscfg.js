
let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");
let soc = system.getScript(`/drivers/mcan/soc/mcan_${common.getSocName()}`);

function getConfigArr() {
    return system.getScript(`/drivers/mcan/soc/mcan_${common.getSocName()}`).getConfigArr();
}

function getInstanceConfig(moduleInstance) {
    let solution = moduleInstance[getInterfaceName(moduleInstance)].$solution;
    let configArr = getConfigArr();
    let config = configArr.find( o => o.name === solution.peripheralName);

    return {
        ...config,
        ...moduleInstance
    }
}

function getPeripheralPinNames(inst) {
    return [ "RX", "TX" ];
}

function pinmuxRequirements(inst) {
   let interfaceName = getInterfaceName(inst);

    let resources = [];
    let pinResource = {};

    pinResource = pinmux.getPinRequirements(interfaceName, "RX", "MCAN RX Pin");
    pinmux.setConfigurableDefault( pinResource, "rx", true );
    resources.push( pinResource);
    pinResource = pinmux.getPinRequirements(interfaceName, "TX", "MCAN TX Pin");
    pinmux.setConfigurableDefault( pinResource, "rx", false );
    resources.push( pinResource);

    let peripheral = {
        name: interfaceName,
        displayName: "MCAN Instance",
        interfaceName: interfaceName,
        resources: resources,
    };

    return [peripheral];
}

function getInterfaceName(instance) {
    return soc.getInterfaceName(instance);
}

function getClockEnableIds(instance) {
    let instConfig = getInstanceConfig(instance);
    return instConfig.clockIds;
}

function getClockFrequencies(inst) {

    let instConfig = getInstanceConfig(inst);

    return instConfig.clockFrequencies;
}

function validate(instance, report) {
    /* None. Verified by SYSCFG based on selected pin */
}

let mcan_module_name = "/drivers/mcan/mcan";

let mcan_module = {
    displayName: "MCAN",
    templates: {
        "/drivers/system/system_config.c.xdt": {
            driver_config: "/drivers/mcan/templates/mcan_config.c.xdt",
        },
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/mcan/templates/mcan.h.xdt",
            moduleName: mcan_module_name,
        },
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: mcan_module_name,
        },
        "/drivers/system/power_clock_config.c.xdt": {
            moduleName: mcan_module_name,
        },
    },
    defaultInstanceName: "CONFIG_MCAN",
    moduleStatic: {
        name: "mcanGlobal",
        displayName: "MCAN Global",
        config: [
            {
                name: "nomRatePrescalar",
                displayName: "Prescalar Value for Nom Bitrate",
                default: 3,
                displayFormat: "dec",
                onChange: function (inst, ui) {
                    inst.nomBitRate = 80000 / (inst.nomRatePrescalar + 1) / (3 + inst.nomTimeSeg1 + inst.nomTimeSeg2)
                }
            },
            {
                name: "nomTimeSeg1",
                displayName: "Time Seg 1 Value for Nom Bitrate",
                default: 15,
                displayFormat: "dec",
                onChange: function (inst, ui) {
                    inst.nomBitRate = 80000 / (inst.nomRatePrescalar + 1) / (3 + inst.nomTimeSeg1 + inst.nomTimeSeg2)
                    inst.samplingNomBitRate = (2 + inst.nomTimeSeg1) / (3 + inst.nomTimeSeg1 + inst.nomTimeSeg2) * 100
                }
            },
            {
                name: "nomTimeSeg2",
                displayName: "Time Seg 2 Value for Nom Bitrate",
                default: 2,
                displayFormat: "dec",
                onChange: function (inst, ui) {
                    inst.nomBitRate = 80000 / (inst.nomRatePrescalar + 1) / (3 + inst.nomTimeSeg1 + inst.nomTimeSeg2)
                    inst.samplingNomBitRate = (2 + inst.nomTimeSeg1) / (3 + inst.nomTimeSeg1 + inst.nomTimeSeg2) * 100
                }
            },
            {
                name: "nomSynchJumpWidth",
                displayName: "Synch Jump Width for Nom Bitrate",
                default: 0,
                displayFormat: "dec",
            },
            {
                name: "nomBitRate",
                displayName: "Effective Nom Bitrate (Kbps)",
                default: 1000,
                description: "Recommended sampling point should be between 85 to 90 percent",
                readOnly: true,
                displayFormat: "dec",
            },
            {
                name: "samplingNomBitRate",
                displayName: "Sampling Point For Nom Bitrate",
                default: 85,
                description: "Recommended sampling point should be between 85 to 90 percent",
                readOnly: true,
                displayFormat: "dec",
            },
            {
                name: "dataRatePrescalar",
                displayName: "Prescalar Value for Data Bitrate",
                default: 1,
                displayFormat: "dec",
                onChange: function (inst, ui) {
                    inst.dataBitRate = 80000 / (inst.dataRatePrescalar + 1) / (3 + inst.dataTimeSeg1 + inst.dataTimeSeg2)
                }
            },
            {
                name: "dataTimeSeg1",
                displayName: "Time Seg 1 Value for Data Bitrate",
                default: 5,
                displayFormat: "dec",
                onChange: function (inst, ui) {
                    inst.dataBitRate = 80000 / (inst.dataRatePrescalar + 1) / (3 + inst.dataTimeSeg1 + inst.dataTimeSeg2)
                    inst.samplingDataBitRate = (2 + inst.dataTimeSeg1) / (3 + inst.dataTimeSeg1 + inst.dataTimeSeg2) * 100
                }
            },
            {
                name: "dataTimeSeg2",
                displayName: "Time Seg 2 Value for Data Bitrate",
                default: 0,
                displayFormat: "dec",
                onChange: function (inst, ui) {
                    inst.dataBitRate = 80000 / (inst.dataRatePrescalar + 1) / (3 + inst.dataTimeSeg1 + inst.dataTimeSeg2)
                    inst.samplingDataBitRate = (2 + inst.dataTimeSeg1) / (3 + inst.dataTimeSeg1 + inst.dataTimeSeg2) * 100
                }
            },
            {
                name: "dataSynchJumpWidth",
                displayName: "Synch Jump Width for Data Bitrate",
                default: 0,
                displayFormat: "dec",
            },
            {
                name: "dataBitRate",
                displayName: "Effective Data Bitrate (Kbps)",
                default: 5000,
                description: "Recommended sampling point should be between 85 to 90 percent",
                readOnly: true,
                displayFormat: "dec",
            },
            {
                name: "samplingDataBitRate",
                displayName: "Sampling Point For Data Bitrate",
                default: 87.5,
                description: "Recommended sampling point should be between 85 to 90 percent",
                readOnly: true,
                displayFormat: "dec",
            },
        ],

        validate : function (instance, report) {
            common.validate.checkNumberRange(instance, report, "nomRatePrescalar", 0, 511, "dec");
            common.validate.checkNumberRange(instance, report, "nomTimeSeg1", 1, 255, "dec");
            common.validate.checkNumberRange(instance, report, "nomTimeSeg2", 0, 127, "dec");
            common.validate.checkNumberRange(instance, report, "nomSynchJumpWidth", 0, 127, "dec");

            common.validate.checkNumberRange(instance, report, "dataRatePrescalar", 0, 31, "dec");
            common.validate.checkNumberRange(instance, report, "dataTimeSeg1", 0, 31, "dec");
            common.validate.checkNumberRange(instance, report, "dataTimeSeg2", 0, 15, "dec");
            common.validate.checkNumberRange(instance, report, "dataSynchJumpWidth", 0, 15, "dec");
        },
    },

    validate: validate,
    modules: function(instance) {
        return [{
            name: "system_common",
            moduleName: "/system_common",
        }]
    },
    getInstanceConfig,
    pinmuxRequirements,
    getInterfaceName,
    getPeripheralPinNames,
    getClockEnableIds,
    getClockFrequencies,
    onMigrate,
};

function onMigrate(newInst, oldInst, oldSystem) {
    let pins = getPeripheralPinNames(oldInst);
    let interfaceName = getInterfaceName(oldInst);
    common.onMigrate(newInst, oldInst, oldSystem, pins, interfaceName)
}

exports = mcan_module;
