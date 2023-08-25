
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

    let iep = getPeripheralRequirements(inst, "IEP");

    /* set default values for "rx" for different pins, based on use case */
    pinmux.setPeripheralPinConfigurableDefault( iep, "EDC_LATCH_IN0", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault( iep, "EDC_LATCH_IN1", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault( iep, "EDC_SYNC_OUT0", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault( iep, "EDC_SYNC_OUT1", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault( iep, "EDIO_DATA_IN_OUT28", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault( iep, "EDIO_DATA_IN_OUT29", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault( iep, "EDIO_DATA_IN_OUT30", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault( iep, "EDIO_DATA_IN_OUT31", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault( iep, "EDIO_OUTVALID", "rx", false);

    return [iep];
}

function getInterfaceNameList(inst) {

    return [
        getInterfaceName(inst, "IEP"),
    ];
}

function getPeripheralPinNames(inst)
{
    let pinList = [];

    pinList = pinList.concat(getInterfacePinList(inst, "IEP"));
    return pinList;
}

let pruicss_top_module_name = "/drivers/pruicss/g_v0/pruicss_g_v0_gpio_iep";

let pruicss_top_module = {
    displayName: "PRU (ICSS) IEP IO",

    templates: {
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: pruicss_top_module_name,
        },
    },

    defaultInstanceName: "CONFIG_PRU_ICSS_IEP_IO",
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
