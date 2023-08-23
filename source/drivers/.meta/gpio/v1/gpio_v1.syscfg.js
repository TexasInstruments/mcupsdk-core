
let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");
let soc = system.getScript(`/drivers/gpio/soc/gpio_${common.getSocName()}`);

function getConfigArr() {
    return soc.getConfigArr();
}

function getInstanceConfig(moduleInstance) {
    let configArr = getConfigArr();
    let config = configArr.find( o => o.name === moduleInstance.instance);
    let additionalConfig =  {
        pinIndex: getPinIndex(moduleInstance),
    }

     return {
        ...config,
        ...moduleInstance,
        ...additionalConfig,
     };
};

function pinmuxRequirements(inst) {
    let resources = [];
    let interfaceName = inst.instance;

    switch(interfaceName)
    {
        case "MSS_GPIO":
        case "GPIO":
            resources.push( pinmux.getGpioPinRequirements(interfaceName,"GPIO0") );
            break;

        case "RCSS_GPIO":
            resources.push( pinmux.getGpioPinRequirements(interfaceName,"GPIO32") );
            break;
    }
    let peripheral = {
        name          : interfaceName,
        displayName   : "GPIO Peripheral",
        interfaceName : interfaceName,
        resources : resources,
        canShareWith: "/drivers/gpio/gpio",
    };

    return [peripheral];
}

function getPinIndex(moduleInstance) {
    let peripheralPinName;
    let interfaceName = getInterfaceName(moduleInstance);

    peripheralPinName = moduleInstance[interfaceName].gpioPin.$solution.peripheralPinName;

    if(! peripheralPinName)
        return "INVALID";

    let splitStrings =  peripheralPinName.split("GPIO");

    /* The last split is the GPIO number */
    return splitStrings[splitStrings.length-1];
}

function getInterfaceName(inst) {
    return inst.instance;
}

function getPeripheralPinNames(inst) {
    return [ "gpioPin" ];
}

function validate(inst, report) {
    /* None. Verified by SYSCFG based on selected pin */
}

let gpio_module_name = "/drivers/gpio/gpio";

let gpio_module = {
    displayName: "GPIO",
    templates: {
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/gpio/templates/gpio_v1.h.xdt",
            moduleName: gpio_module_name,
        },
        "/drivers/system/system_config.c.xdt": {
            driver_config: "/drivers/gpio/templates/gpio_config_v1.c.xdt",
            driver_init: "/drivers/gpio/templates/gpio_init.c.xdt",
            driver_deinit: "/drivers/gpio/templates/gpio_deinit.c.xdt",
            moduleName: gpio_module_name,
        },
        "/drivers/system/drivers_open_close.c.xdt": {
            driver_open_close_config: "/drivers/gpio/templates/gpio_open_close_config_v1.c.xdt",
            driver_open: "/drivers/gpio/templates/gpio_open.c.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt": {
            driver_open_close_config: "/drivers/gpio/templates/gpio_open_close.h.xdt",
        },
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: gpio_module_name,
        },
    },
    defaultInstanceName: "CONFIG_GPIO",
    config: [
        {
            name: "pinDir",
            displayName: "PIN Direction",
            default: "INPUT",
            options: [
                {
                    name: "INPUT",
                    displayName: "Input"
                },
                {
                    name: "OUTPUT",
                    displayName: "Output"
                },
            ],
            description: "GPIO PIN direction",
        },
        {
            name: "trigType",
            displayName: "Trigger Type",
            default: "NONE",
            options: [
                {
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
                    displayName: "Both Edge",
                },
            ],
            description: "GPIO PIN Trigger Type",
        },
        {
            name: "trigLevel",
            displayName: "Trigger Level",
            default: "LOW",
            options: [
                {
                    name: "LOW",
                    displayName: "Low",
                },
                {
                    name: "HIGH",
                    displayName: "High",
                },
            ],
            description: "GPIO PIN Trigger Level",
        },
        {
            name: "openDrain",
            displayName: "Open Drain",
            default: "DISABLE",
            options: [
                {
                    name: "ENABLE",
                    displayName: "Enable",
                },
                {
                    name: "DISABLE",
                    displayName: "Disable",
                },
            ],
            description: "GPIO PIN Open Drain Status",
        },
        common.ui.makeInstanceConfig(getConfigArr()),
    ],
    validate: validate,
    modules: function(inst) {
        return [{
            name: "system_common",
            moduleName: "/system_common",
        }]
    },
    pinmuxRequirements,
    getInstanceConfig,
    getInterfaceName,
    getPeripheralPinNames,
};

exports = gpio_module;
