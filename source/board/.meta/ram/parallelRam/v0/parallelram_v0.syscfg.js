let common = system.getScript("/common");
let soc = system.getScript(`/board/ram/parallelRam/soc/parallelram_${common.getSocName()}`);

function getDriver(drvName) {
    return system.getScript(`/drivers/${drvName}/${drvName}`);
}

function getInstanceConfig(moduleInstance) {

    return {
        ...moduleInstance,
    };
};
let defaultDevice = soc.getDefaultDevice();

let parallelRam_module = {
    displayName: defaultDevice,
    collapsed: false,
    config: [
        {
            name: "pname",
            displayName: defaultDevice + " Name",
            default: soc.getDefaultPsramName(),
            placeholder: "Type your psram name here",
        },
        {
            name: "deviceWidth",
            displayName: "Device Size",
            default: "16 bit",
            options: [
                { name: "16 bit" },
            ],
        },
        /* Psram Config */
        {
            name: "basicPramCfg",
            displayName: "Basic " + defaultDevice + " Configuration",
            collapsed: true,
            config: [
                {
                    name: "pramSize",
                    displayName: defaultDevice + " Size In Bytes",
                    default: soc.getDefaultPsramConfig().psramSize,
                    displayFormat: "dec",
                },
            ]
        }
    ],
    moduleStatic: {
        modules: function(inst) {
            return [{
                name: "system_common",
                moduleName: "/system_common",
            }]
        },
    },
    validate: validate,
    moduleInstances: moduleInstances,
    getInstanceConfig,
};

function validate(inst, report) {

    common.validate.checkSameFieldName(inst, "device", report);

}

function moduleInstances(inst) {

    let modInstances = new Array();
    let requiredArgs = {
        deviceWidth: inst.deviceWidth,
        deviceType: "PSRAM"
    };

    let requiredArgsGpio = {
        $name: "CONFIG_PSRAM_GPIO_ZZ",
        pinDir: "OUTPUT",
    };


    if(common.getSocName() ==  "am263x")
    {
        modInstances.push({
            name: "psramDriverIoExpander",
            displayName: defaultDevice + " Configuration",
            moduleName: "/drivers/i2c/i2c",
            useArray: false,
        })
    }

    modInstances.push({
        name: "sleepEnGpioDriver",
        displayName: "PSRAM GPIO Configuration",
        moduleName: '/drivers/gpio/gpio',
        requiredArgs: requiredArgsGpio,
        });

    modInstances.push({
            name: "psramDriver",
            displayName: defaultDevice + " Configuration",
            moduleName: "/drivers/gpmc/gpmc",
            requiredArgs: requiredArgs,
            useArray: false,
        })

    return (modInstances);
}

exports = parallelRam_module;