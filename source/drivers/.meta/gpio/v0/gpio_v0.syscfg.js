let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");
let soc = system.getScript(`/drivers/gpio/soc/gpio_${common.getSocName()}`);
let boardConfig = system.getScript(`/drivers/gpio/soc/k3BoardConfig.json`)

function getInstanceConfig(moduleInstance) {
    let additionalConfig = {
        baseAddr: `CSL_${soc.getInstanceString(moduleInstance)}_BASE`,
        moduleIndex: soc.getInstanceString(moduleInstance),
        pinIndex: soc.getPinIndex(moduleInstance),
        CoreID: getSelfCoreID(),
        Cfg:boardConfig,
    }

    return {
        ...moduleInstance,
        ...additionalConfig,
    };
};

//function to get Core ID of the core.
function getSelfCoreID() {
    return system.getScript(`/drivers/soc/drivers_${common.getSocName()}`).getSelfCoreID();
}

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

function validate(inst, report){
    validateInterruptRouter(inst, report , "intrOut");
}


//Function to validate if same interrupt router is selected for other instances
function validateInterruptRouter(instance, report,fieldname) {
    /* Verified by SYSCFG based on selected pin */
    if(instance.advanced){
        let moduleInstances = instance.$module.$instances;
        let validOptions = instance.$module.$configByName.intrOut.options(instance);
        let selectedOptions = instance.intrOut;
        let found = _.find(validOptions, (o) => o.name === selectedOptions)
        if (!found) {
            report[`log${"Error"}`]("Selected option is invalid, please reselect.", instance, fieldname);
        }
        for (let i = 0; i < moduleInstances.length; i++) {
            if (instance[fieldname] === moduleInstances[i][fieldname] &&
                instance !== moduleInstances[i]) {
                report.logError("Same Interrupt Router lines cannot be selected",instance , fieldname);
                return
            }
        }
    }
}

//To  get long description
function getLongDescription(data) {
    let routerDescription = `The interrupt router input to the Core are shared for different resources. Although many output pins were available for the GPIO MUX interrupt router, only resource pin that are allocated in board configuration is available for use`;
    let buttonDescription = 'If you manually changed the resource management (RM) data in source/drivers/sciclient/sciclient_default_boardcfg/am64x_am243x/sciclient_defaultBoardcfg_rm.c, click this button to reflect it in SysConfig.'
    if (data=="router"){
        return routerDescription;
    }
    if(data=="button"){
        return buttonDescription;
    }
    return "";
}

//function to get router pin data from boardConfig
function getRouterPins() {
    let soc_name = common.getSocName();
    if(soc_name =="am64x" || soc_name =="am243x"){
        soc_name="am64x_am243x"
    }
    let core = common.getSelfSysCfgCoreName();
    return boardConfig[soc_name][core]["outPinCfg"];
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
    }, {
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
    )

    if (common.isMcuDomainSupported()) {
        config.push(
            {
                name: "advanced",
                displayName: "Enable Interrupt Configuration",
                default: false,
                onChange: function (inst, ui) {
                    let hideConfigs = true;
                    if (inst.advanced == true) {
                        hideConfigs = false;
                    }
                    ui.intrOut.hidden = hideConfigs;
                    ui.getBoardCfg.hidden = hideConfigs;
                },
            },
            {
                name: "getBoardCfg",
                displayName: "Fetch Board Configuration",
                description: 'Click this button to fetch GPIO RM data automatically',
                longDescription: getLongDescription("button"),
                buttonText: "GET RM DATA",
                hidden: true,
                onLaunch: (inst) => {
                    let products=system.getProducts()
                    let sdk_path = products[0].path.split("/.metadata/product.json")[0]
                    let file_path=sdk_path + "/source/drivers/.meta/gpio/soc/getBoardConfigRm.js"
                    if(system.getOS()=="win"){
                        sdk_path = products[0].path.split("\\.metadata\\product.json")[0]
                        file_path = sdk_path+ "//source//drivers//.meta//gpio//soc//getBoardConfigRm.js"
                    }
                    return {
                        command:"node",
                        args: [file_path, "$comFile"],
                        initialData: "initialData",
                        inSystemPath: true,
                    };
                },
                onComplete: (inst, _ui, result) => {
                    if(result.data === "error") {
                        return;
                    } else if(result.data === "initialData") {
                        return;
                    } else {
                        try {
                            boardConfig = JSON.parse(result.data);
                        } catch (e) {
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
                longDescription: getLongDescription("router"),
                hidden: true,
                default: "",
                options: getRouterPins,
            },
        )
    }
    if (common.isMcuDomainSupported()) {
        config.push(common.getUseMcuDomainPeripheralsConfig());
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