let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");

function getInterfaceName(inst, peripheralName)
{
    return `PRU_${inst.icssInstance}_${peripheralName}`;
}

function getPeripheralRequirements(inst, peripheralName)
{
    let interfaceName = getInterfaceName(inst, peripheralName);
    let resources = [];

    // ADC Control & Communication Pins
    if (peripheralName=="PRU")
    {
        resources.push(pinmux.getPinRequirements(interfaceName, "GPO8", "C̅S̅ "));
        resources.push(pinmux.getPinRequirements(interfaceName, "GPO9", "DOUT "));
        resources.push(pinmux.getPinRequirements(interfaceName, "GPI14", "DIN "));
        resources.push(pinmux.getPinRequirements(interfaceName, "GPI16", "DRDY "));
        resources.push(pinmux.getPinRequirements(interfaceName, "GPO19", "SCLK "));
    }

    for(let pinResource of resources)
    {
        /* make all pins as "rx" and then override to make "rx" as false as needed  */
        pinmux.setConfigurableDefault(pinResource, "rx", false);
        /* make all pins as "pd" and then override to make them "pu" or "nopull" as needed  */
        pinmux.setConfigurableDefault(pinResource, "pu_pd", "nopull");

        /* Enable all the pins. */
        pinResource.used=true;
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

    /* set default values for "rx" and "pu_pd" for different pins, based on use case */
    pinmux.setPeripheralPinConfigurableDefault(pru, "GPI14", "rx", true);
    pinmux.setPeripheralPinConfigurableDefault(pru, "GPI16", "rx", true);
    pinmux.setPeripheralPinConfigurableDefault(pru, "GPO8", "pu_pd", "pu");

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

    let pinReq = pinmuxRequirements(inst);
    for(let peripheralReq of pinReq)
    {
        for(let resource of peripheralReq.resources)
        {
            pinList.push(resource.name);
        }
    }
    return pinList;
}

function moduleInstances(instance) {
    // if the pin is not being used or hardwired just unselect the pin
    let modInstances =  [{
            name: "power",
            displayName: "Power Enable Pin",
            moduleName: "/drivers/gpio/gpio",
            requiredArgs: {
                pinDir: "OUTPUT",
                GPIO: {
                    gpioPin: {
                        pu_pd: "pd",
                    },
                },
                useMcuDomainPeripherals: false,
            },
        },
        {
            name: "rst",
            displayName: "RESET",
            moduleName: "/drivers/gpio/gpio",
            requiredArgs: {
                pinDir: "OUTPUT",
                useMcuDomainPeripherals: false,
            },
            args: {
                GPIO: {
                    gpioPin: {
                        pu_pd: "pu",    // reset ADC in starting
                    },
                },
            },
        },
    ]

    modInstances.push({
        name: "pruIpc",
        displayName: "PRU IPC",
        moduleName: '/pru_io/pru_ipc/pru_ipc',
        collapsed: false,
        useArray: true,
        minInstanceCount : 1,
        defaultInstanceCount: 1,
        args: {
            icssInstance: instance.icssInstance,
            noOfBuffers: parseInt(instance.channelsInUse),
            pruCore: "PRU0",
        },
    });

    return (modInstances);

}

let adc_phi_pru_evm_adapter_top_module_name = "/pru_io/adc/ads131/adc_phi_pru_evm_adapter";

let adc_phi_pru_evm_adapter_top_module = {
    displayName: "ADC Configuration",

    templates: {
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: adc_phi_pru_evm_adapter_top_module_name,
        },
    },
    defaultInstanceName: "CONFIG_ADS131_PINMUX",
    config: [
        {
            name: "icssInstance",
            displayName: "ICSSG Instance",
            default: "ICSSG0",
            options: [
                {
                    name: "ICSSG0",
                },
                {
                    name: "ICSSG1",
                }
            ],
            hidden: true,
        },
        {
            name: "interface",
            displayName: "Interface With ADC",
            options: [{
                name: "serial",
                displayName: "Serial",
            }],
            default: "serial",
            readOnly: true,
        },
        {
            name: "channelsInUse",
            displayName: "Channels In Use",
            description: "Number of adc channels to use for sampling from 1 to n",
            options: [
                { name: "8", },
            ],
            default: "8",
        },
    ],
    pinmuxRequirements,
    getInterfaceNameList,
    getPeripheralPinNames,
    sharedModuleInstances,
    moduleInstances,
};

function validate(inst, report) {
    common.validate.checkSameInstanceName(inst, report);
}

function sharedModuleInstances(instance) {
    let modInstances = new Array();

    modInstances.push({
        name: "icss",
        displayName: "PRU Configuration",
        moduleName: '/drivers/pruicss/pruicss',
        requiredArgs: {
            instance: instance.icssInstance,
            // clock speeds fixed for now
            coreClk: 333333333,
            iepSyncMode: false,
            iepClk: 500*1000000,
        },
    });

    return (modInstances);
}

exports = adc_phi_pru_evm_adapter_top_module;