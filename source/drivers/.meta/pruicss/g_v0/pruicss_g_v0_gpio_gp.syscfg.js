
let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");

function getInterfaceName(inst, peripheralName)
{
    return `PRU_${inst.instance}_${peripheralName}`;
}

function getInterfacePinList(inst, peripheralName)
{
    let interfaceName = getInterfaceName(inst, peripheralName);
    let pinList = [];

    pinList = pinmux.getInterfacePinList(interfaceName);

    return pinList;
}

function getPeripheralRequirements(inst, peripheralName)
{
    let interfaceName = getInterfaceName(inst, peripheralName);
    let pinList = getInterfacePinList(inst, peripheralName);
    let resources = [];
    let device = common.getDeviceName();

    for(let pin of pinList)
    {
        let pinResource = pinmux.getPinRequirements(interfaceName, pin);

        /* make all pins as "rx" and then override to make "rx" as false as needed  */
        pinmux.setConfigurableDefault( pinResource, "rx", true );

        /* Disable all the pins. */
        pinResource.used=false;

        resources.push( pinResource );
    }

    let peripheralRequirements = {
        name: interfaceName,
        displayName: interfaceName,
        interfaceName: interfaceName,
        resources: resources,
    };

    return peripheralRequirements;
}

function pinmuxRequirements(inst) {

    let pru = getPeripheralRequirements(inst, "PRU");

    /* set default values for "rx" for different pins, based on use case */
    pinmux.setPeripheralPinConfigurableDefault( pru, "GPI0", "rx", true);
    pinmux.setPeripheralPinConfigurableDefault( pru, "GPI1", "rx", true);
    pinmux.setPeripheralPinConfigurableDefault( pru, "GPI2", "rx", true);
    pinmux.setPeripheralPinConfigurableDefault( pru, "GPI3", "rx", true);
    pinmux.setPeripheralPinConfigurableDefault( pru, "GPI4", "rx", true);
    pinmux.setPeripheralPinConfigurableDefault( pru, "GPI5", "rx", true);
    pinmux.setPeripheralPinConfigurableDefault( pru, "GPI6", "rx", true);
    pinmux.setPeripheralPinConfigurableDefault( pru, "GPI7", "rx", true);
    pinmux.setPeripheralPinConfigurableDefault( pru, "GPI8", "rx", true);
    pinmux.setPeripheralPinConfigurableDefault( pru, "GPI9", "rx", true);
    pinmux.setPeripheralPinConfigurableDefault( pru, "GPI10", "rx", true);
    pinmux.setPeripheralPinConfigurableDefault( pru, "GPI11", "rx", true);
    pinmux.setPeripheralPinConfigurableDefault( pru, "GPI12", "rx", true);
    pinmux.setPeripheralPinConfigurableDefault( pru, "GPI13", "rx", true);
    pinmux.setPeripheralPinConfigurableDefault( pru, "GPI14", "rx", true);
    pinmux.setPeripheralPinConfigurableDefault( pru, "GPI15", "rx", true);
    pinmux.setPeripheralPinConfigurableDefault( pru, "GPI16", "rx", true);
    pinmux.setPeripheralPinConfigurableDefault( pru, "GPI17", "rx", true);
    pinmux.setPeripheralPinConfigurableDefault( pru, "GPI18", "rx", true);
    pinmux.setPeripheralPinConfigurableDefault( pru, "GPI19", "rx", true);

    pinmux.setPeripheralPinConfigurableDefault( pru, "GPO0", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault( pru, "GPO1", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault( pru, "GPO2", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault( pru, "GPO3", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault( pru, "GPO4", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault( pru, "GPO5", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault( pru, "GPO6", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault( pru, "GPO7", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault( pru, "GPO8", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault( pru, "GPO9", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault( pru, "GPO10", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault( pru, "GPO11", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault( pru, "GPO12", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault( pru, "GPO13", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault( pru, "GPO14", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault( pru, "GPO15", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault( pru, "GPO16", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault( pru, "GPO17", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault( pru, "GPO18", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault( pru, "GPO19", "rx", false);

    return [pru];
}

function getInterfaceNameList(inst) {

    return [
        getInterfaceName(inst, "PRU"),
    ];
}

function getPeripheralPinNames(inst)
{
    let pinList = [];

    pinList = pinList.concat(getInterfacePinList(inst, "PRU"));
    return pinList;
}

let pruicss_top_module_name = "/drivers/pruicss/g_v0/pruicss_g_v0_gpio_gp";

let pruicss_top_module = {
    displayName: "PRU (ICSS) GPIO",

    templates: {
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: pruicss_top_module_name,
        },
    },

    defaultInstanceName: "CONFIG_PRU_ICSS_GPIO",
    config: [
        {
            name: "instance",
            displayName: "Instance",
            default: "ICSSG0",
            options: [
                {
                    name: "ICSSG0",
                },
                {
                    name: "ICSSG1",
                }
            ],
        },
    ],
    pinmuxRequirements,
    getInterfaceNameList,
    getPeripheralPinNames,
};

function validate(inst, report) {
    common.validate.checkSameInstanceName(inst, report);
}

exports = pruicss_top_module;
