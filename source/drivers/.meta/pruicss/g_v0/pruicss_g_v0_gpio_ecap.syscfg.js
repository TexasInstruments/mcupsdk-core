
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

	let ecap = getPeripheralRequirements(inst, "ECAP");

    /* set default values for "rx" for different pins, based on use case */
    pinmux.setPeripheralPinConfigurableDefault( ecap, "IN_APWM_OUT", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault( ecap, "SYNC_IN", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault( ecap, "SYNC_OUT", "rx", false);

    return [ecap];
}



function getInterfaceNameList(inst) {

    return [
        getInterfaceName(inst, "ECAP"),
    ];
}

function getPeripheralPinNames(inst)
{
    let pinList = [];

    pinList = pinList.concat(getInterfacePinList(inst, "ECAP"));
    return pinList;
}

let pruicss_top_module_name = "/drivers/pruicss/g_v0/pruicss_g_v0_gpio_ecap";

let pruicss_top_module = {
    displayName: "PRU (ICSS) ECAP IO",

    templates: {
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: pruicss_top_module_name,
        },
    },

    defaultInstanceName: "CONFIG_PRU_ICSS_ECAP_IO",
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
