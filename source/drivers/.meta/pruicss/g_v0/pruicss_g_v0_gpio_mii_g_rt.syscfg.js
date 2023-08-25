
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

    let mii_g_rt = getPeripheralRequirements(inst, "MII_G_RT");

    /* set default values for "rx" for different pins, based on use case */
    pinmux.setPeripheralPinConfigurableDefault( mii_g_rt, "MII0_TXD0", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault( mii_g_rt, "MII0_TXD1", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault( mii_g_rt, "MII0_TXD2", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault( mii_g_rt, "MII0_TXD3", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault( mii_g_rt, "MII0_TXEN", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault( mii_g_rt, "MII1_TXD0", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault( mii_g_rt, "MII1_TXD1", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault( mii_g_rt, "MII1_TXD2", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault( mii_g_rt, "MII1_TXD3", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault( mii_g_rt, "MII1_TXEN", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault( mii_g_rt, "MII_MT0_CLK", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault( mii_g_rt, "MII_MT1_CLK", "rx", false);

    return [mii_g_rt];
}

function getInterfaceNameList(inst) {

    return [
        getInterfaceName(inst, "MII_G_RT"),
    ];
}

function getPeripheralPinNames(inst)
{
    let pinList = [];

    pinList = pinList.concat(getInterfacePinList(inst, "MII_G_RT"));
    return pinList;
}

let pruicss_top_module_name = "/drivers/pruicss/g_v0/pruicss_g_v0_gpio_mii_g_rt";

let pruicss_top_module = {
    displayName: "PRU (ICSS) MII_G_RT IO",

    templates: {
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: pruicss_top_module_name,
        },
    },

    defaultInstanceName: "CONFIG_PRU_ICSS_MII_G_RT_IO",
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
