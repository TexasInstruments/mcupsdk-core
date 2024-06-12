let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");
let soc = system.getScript(`/drivers/gpio/soc/gpio_${common.getSocName()}`);
let boardConfig = system.getScript(`/drivers/gpio/soc/k3BoardConfig.json`);

let errorFlag = 0;
let errorLog = '';

function getInstanceConfig(moduleInstance) {
    let coreId = undefined;
    let routerId = undefined;
    if (common.isMcuDomainSupported() || common.isWakeupDomainSupported()) {
        coreId = soc.getTisciDestCoreID();
        routerId = soc.getCpuRouterId();
    }
    let additionalConfig = {
        baseAddr: `CSL_${soc.getInstanceString(moduleInstance)}_BASE`,
        moduleIndex: soc.getInstanceString(moduleInstance),
        pinIndex: soc.getPinIndex(moduleInstance),
    }

    return {
        ...moduleInstance,
        ...additionalConfig,
        coreId,
        routerId,
    };
};

function getInterfaceName(inst) {
    return soc.getInterfaceName(inst);
}

function pinmuxRequirements(inst) {
    let interfaceName = getInterfaceName(inst);

    let resources = [];
    resources.push(pinmux.getGpioPinRequirements(interfaceName, "0"));

    let peripheral = {
        name: interfaceName,
        displayName: "GPIO Peripheral",
        interfaceName: interfaceName,
        resources: resources,
        canShareWith: "/drivers/gpio/gpio",
    };

    return [peripheral];
}

function getPeripheralPinNames(inst) {
    return ["gpioPin"];
}

function validate(inst, report) {
        validateInterruptRouter(inst, report, "intrOut");
}


//Function to validate if same interrupt router is selected for other instances
function validateInterruptRouter(instance, report, fieldname) {
    /* Verified by SYSCFG based on selected pin */
    if (instance.enableIntr && common.isSciClientSupported()) {
        common.validate.checkNumberRange(instance, report, fieldname, 0, soc.getMaxInterruptRouters(), "dec");
        if(errorFlag == 1) {
            report.logWarning(errorLog, instance, fieldname);
        }
        let moduleInstances = instance.$module.$instances;
        let validOptions = instance.$module.$configByName.intrOut.options(instance);
        let selectedOptions = instance.intrOut;
        let found = _.find(validOptions, (o) => o.name === selectedOptions)
        if (found === undefined || found === null) {
            report.logError("Selected option is invalid, please reselect.", instance, fieldname);
        }
        for (let i = 0; i < moduleInstances.length; i++) {
            if (instance[fieldname] === moduleInstances[i][fieldname] && instance !== moduleInstances[i]
                && instance.enableIntr === true && moduleInstances[i].enableIntr === true) {
                report.logError("Same Interrupt Router lines cannot be selected", instance, fieldname);
                return
            }
        }
    }
}

let routerDescription =
`The interrupt router input to the Core are shared for different resources.
 Although many output pins were available for the GPIO MUX interrupt router,
 only resource pin that are allocated in board configuration is available for use`;
let buttonDescription =
`If you manually changed the resource management (RM) data in
source/drivers/sciclient/sciclient_default_boardcfg/am64x_am243x/sciclient_defaultBoardcfg_rm.c,
click this button to reflect it in SysConfig`;

//function to get router pin data from boardConfig
function getRouterPins() {
    let socName = common.getSocName();
    let core = common.getSelfSysCfgCoreName();
    return boardConfig[socName][core]["outPinCfg"];
}

function getConfigurables() {
    let config = [];
    config.push({
        name: "pinDir",
        displayName: "PIN Direction",
        default: "INPUT",
        options: [{
            name: "INPUT",
            displayName: "Input"
        },
        {
            name: "OUTPUT",
            displayName: "Output"
        },
        ],
        description: "GPIO PIN direction",
        onChange: function(inst, ui) {
            if(inst.pinDir == "OUTPUT"){
                ui.defaultValue.hidden = false;
            }
            else {
                ui.defaultValue.hidden = true;
            }
        }
    },
    {
        name: "defaultValue",
        displayName: "Default Value",
        default: "0",
        options: [
            { name: "0" },
            { name: "1" },
        ],
        description: "Default value of GPIO OUT register",
        hidden: true,
    },
    {
        name: "trigType",
        displayName: "Trigger Type",
        default: "NONE",
        options: [{
            name: "NONE",
            displayName: "None",
        },
        {
            name: "RISE_EDGE",
            displayName: "Rising Edge",
        },
        {
            name: "FALL_EDGE",
            displayName: "Falling Edge",
        },
        {
            name: "BOTH_EDGE",
            displayName: "Rising and Falling",
        },
        ],
        description: "GPIO PIN Trigger Type",
    },
    {
        name: "enableIntr",
        displayName: "Enable Interrupt Configuration",
        default: false,
        onChange: function (inst, ui) {
            if (common.isSciClientSupported())
            {
                let hideConfigs = true;
                if (inst.enableIntr == true) {
                    hideConfigs = false;
                }
                ui.intrOut.hidden = hideConfigs;
                ui.getBoardCfg.hidden = hideConfigs;
            }
        },
    },
    )
    if (common.isSciClientSupported()) {
        config.push(
            {
                name: "getBoardCfg",
                displayName: "Fetch Board Configuration",
                description: 'Click this button to fetch GPIO RM data automatically',
                longDescription: buttonDescription,
                buttonText: "GET RM DATA",
                hidden: true,
                onLaunch: (inst) => {
                    let nodeCmd = common.getNodePath()
                    let products = system.getProducts()
                    let sdkPath = products[0].path.split("/.metadata/product.json")[0]
                    let filePath = sdkPath + "/source/drivers/.meta/gpio/soc/getBoardConfigRm.js"
                    let socName = common.getSocName();
                    if (system.getOS() == "win") {
                        sdkPath = products[0].path.split("\\.metadata\\product.json")[0]
                        filePath = sdkPath + "//source//drivers//.meta//gpio//soc//getBoardConfigRm.js"
                    }
                    return {
                        command: nodeCmd,
                        args: [filePath, "$comFile", socName],
                        initialData: "initialData",
                        inSystemPath: true,
                    };
                },
                onComplete: (inst, _ui, result) => {
                    if (result.data === "error") {
                        return;
                    } else if (result.data === "initialData") {
                        return;
                    } else {
                        try {
                            errorFlag = 0;
                            boardConfig = JSON.parse(result.data);
                        } catch (e) {
                            errorFlag = 1;
                            errorLog = result.data;
                            return;
                        }
                        return;
                    }
                },
            },
            {
                name: "intrOut",
                displayName: "Interrupt Router Output",
                description: 'GPIO-MUX interrupt Router Output to the destination Core',
                longDescription: routerDescription,
                hidden: true,
                default: getRouterPins()[0].name,
                options: getRouterPins,
            },
        )
    }
    if ((common.isMcuDomainSupported()) && (common.getSocName() != "am65x")) {
        config.push(common.getUseMcuDomainPeripheralsConfig());
    }
    if(common.isWakeupDomainSupported())
    {
      config.push(common.getUseWakeupDomainPeripheralsConfig());
    }
    return config;
}

let gpio_module_name = "/drivers/gpio/gpio";

let gpio_module = {
    displayName: "GPIO",
    templates: {
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/gpio/templates/gpio.h.xdt",
            moduleName: gpio_module_name,
        },
        "/drivers/system/system_config.c.xdt": {
            driver_config: "/drivers/gpio/templates/gpio_config.c.xdt",
            driver_init: "/drivers/gpio/templates/gpio_init.c.xdt",
            driver_deinit: "/drivers/gpio/templates/gpio_deinit.c.xdt",
            moduleName: gpio_module_name,
        },
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: gpio_module_name,
        },
    },
    defaultInstanceName: "CONFIG_GPIO",
    config: getConfigurables(),
    validate: validate,
    modules: function (inst) {
        return [{
            name: "system_common",
            moduleName: "/system_common",
        }]
    },
    getInstanceConfig,
    pinmuxRequirements,
    getInterfaceName,
    getPeripheralPinNames,
};

exports = gpio_module;