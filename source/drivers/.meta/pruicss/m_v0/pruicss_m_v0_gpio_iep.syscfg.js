
let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");

function getInterfaceName(inst, peripheralName)
{
    return `${inst.instance}_${peripheralName}`
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

        /* make all pins as "tx" and then override to make "rx" as false as needed  */
        pinmux.setConfigurableDefault( pinResource, "rx", false );

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
    let IEP = getPeripheralRequirements(inst, "IEP");
    return [IEP];
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

let pruicss_top_module_name = "/drivers/pruicss/m_v0/pruicss_m_v0_gpio_iep";

let pruicss_top_module = {
    displayName: "PRU (ICSS) IEP",

    templates: {
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: pruicss_top_module_name,
        },
    },

    defaultInstanceName: "CONFIG_PRU_ICSS_IEP",
    config: [
        {
            name: "instance",
            displayName: "Instance",
            default: "ICSSM",
            options: [
                {
                    name: "ICSSM",
                    displayName: "ICSSM0"
                },
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
