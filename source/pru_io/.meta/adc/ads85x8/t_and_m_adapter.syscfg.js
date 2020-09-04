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
    if (peripheralName=="IEP")
    {
        if (inst.combinedConvst) {
            // CONVST signal: PRU0_GPO19 (IEP0_SYNC_OUT0 output)
            resources.push(pinmux.getPinRequirements(interfaceName, "EDC_SYNC_OUT0", "CONVST (A&B) "));
        } else {
            // CONVST signals: PRU0_GPO19, PRU0_GPO17 (IEP0_SYNC_OUT0, IEP0_SYNC_OUT1)
            resources.push(pinmux.getPinRequirements(interfaceName, "EDC_SYNC_OUT0", "CONVSTA "));
            resources.push(pinmux.getPinRequirements(interfaceName, "EDC_SYNC_OUT1", "CONVSTB "));
        }
    }
    else if (peripheralName=="PRU")
    {
        // DB0-DB15
        if (inst.interface === "parallel") {
            resources.push(pinmux.getPinRequirements(interfaceName, "GPI0",  "DB0 "));
            resources.push(pinmux.getPinRequirements(interfaceName, "GPI1",  "DB1 "));
            resources.push(pinmux.getPinRequirements(interfaceName, "GPI2",  "DB2 "));
            resources.push(pinmux.getPinRequirements(interfaceName, "GPI3",  "DB3 "));
            resources.push(pinmux.getPinRequirements(interfaceName, "GPI4",  "DB4 "));
            resources.push(pinmux.getPinRequirements(interfaceName, "GPI5",  "DB5 "));
            resources.push(pinmux.getPinRequirements(interfaceName, "GPI6",  "DB6 "));
            resources.push(pinmux.getPinRequirements(interfaceName, "GPI7",  "DB7/DOUTA "));
            resources.push(pinmux.getPinRequirements(interfaceName, "GPI8",  "DB8/DOUTB "));
            resources.push(pinmux.getPinRequirements(interfaceName, "GPI9",  "DB9 "));
            resources.push(pinmux.getPinRequirements(interfaceName, "GPI10", "DB10 "));
            resources.push(pinmux.getPinRequirements(interfaceName, "GPI11", "DB11 "));
            resources.push(pinmux.getPinRequirements(interfaceName, "GPI12", "DB12 "));
            resources.push(pinmux.getPinRequirements(interfaceName, "GPI13", "DB13 "));
            resources.push(pinmux.getPinRequirements(interfaceName, "GPI14", "DB14/HBEN "));
            resources.push(pinmux.getPinRequirements(interfaceName, "GPI15", "DB15/BYTE SEL "));
        }
        if (inst.interface === "byteParallel") {
            resources.push(pinmux.getPinRequirements(interfaceName, "GPI0",  "DB0 "));
            resources.push(pinmux.getPinRequirements(interfaceName, "GPI1",  "DB1 "));
            resources.push(pinmux.getPinRequirements(interfaceName, "GPI2",  "DB2 "));
            resources.push(pinmux.getPinRequirements(interfaceName, "GPI3",  "DB3 "));
            resources.push(pinmux.getPinRequirements(interfaceName, "GPI4",  "DB4 "));
            resources.push(pinmux.getPinRequirements(interfaceName, "GPI5",  "DB5 "));
            resources.push(pinmux.getPinRequirements(interfaceName, "GPI6",  "DB6 "));
            resources.push(pinmux.getPinRequirements(interfaceName, "GPI7",  "DB7/DOUTA ")); // DOUTA - Channel 1-4 then Channel 5-8
            resources.push(pinmux.getPinRequirements(interfaceName, "GPO8",  "DB8/DOUTB ")); // DOUTB - Channel 5-8 then Channel 1-4
            resources.push(pinmux.getPinRequirements(interfaceName, "GPO9",  "DB9 "));
            resources.push(pinmux.getPinRequirements(interfaceName, "GPO10", "DB10 "));
            resources.push(pinmux.getPinRequirements(interfaceName, "GPO11", "DB11 "));
            resources.push(pinmux.getPinRequirements(interfaceName, "GPO12", "DB12 "));
            resources.push(pinmux.getPinRequirements(interfaceName, "GPO13", "DB13 "));
            resources.push(pinmux.getPinRequirements(interfaceName, "GPO14", "DB14/HBEN "));
            resources.push(pinmux.getPinRequirements(interfaceName, "GPO15", "DB15/BYTE SEL "));
        }
        if (inst.interface === "serial") {
            resources.push(pinmux.getPinRequirements(interfaceName, "GPO0",  "DB0 "));
            resources.push(pinmux.getPinRequirements(interfaceName, "GPO1",  "DB1 "));
            resources.push(pinmux.getPinRequirements(interfaceName, "GPO2",  "DB2 "));
            resources.push(pinmux.getPinRequirements(interfaceName, "GPO3",  "DB3 "));
            resources.push(pinmux.getPinRequirements(interfaceName, "GPO4",  "DB4 "));
            resources.push(pinmux.getPinRequirements(interfaceName, "GPO5",  "DB5 "));
            resources.push(pinmux.getPinRequirements(interfaceName, "GPO6",  "DB6 "));
            resources.push(pinmux.getPinRequirements(interfaceName, "GPI7",  "DB7/DOUTA "));
            resources.push(pinmux.getPinRequirements(interfaceName, "GPI8",  "DB8/DOUTB "));
            resources.push(pinmux.getPinRequirements(interfaceName, "GPO9",  "DB9 "));
            resources.push(pinmux.getPinRequirements(interfaceName, "GPO10", "DB10 "));
            resources.push(pinmux.getPinRequirements(interfaceName, "GPO11", "DB11 "));
            resources.push(pinmux.getPinRequirements(interfaceName, "GPO12", "DB12 "));
            resources.push(pinmux.getPinRequirements(interfaceName, "GPO13", "DB13 "));
            resources.push(pinmux.getPinRequirements(interfaceName, "GPO14", "DB14/HBEN "));
            resources.push(pinmux.getPinRequirements(interfaceName, "GPO15", "DB15/BYTE SEL "));
        }
        // BUSY
        resources.push(pinmux.getPinRequirements(interfaceName, "GPI16", "BUSY "));
        // FIRST
        resources.push(pinmux.getPinRequirements(interfaceName, "GPI17", "FRSTDATA "));
        // CS
        resources.push(pinmux.getPinRequirements(interfaceName, "GPO18", "C̅S̅ "));
        // READ
        resources.push(pinmux.getPinRequirements(interfaceName, "GPO19", "R̅D̅/SCLK "));
    } else
    {
        // interfaceName = "GPIO";
        // resources.push(pinmux.getPinRequirements("GPIO", "GPIO19", "R̅D̅/SCLK "));
    }

    for(let pinResource of resources)
    {
        /* make all pins as "rx" and then override to make "rx" as false as needed  */
        pinmux.setConfigurableDefault(pinResource, "rx", true);
        /* make all pins as "pd" and then override to make them "pu" or "nopull" as needed  */
        pinmux.setConfigurableDefault(pinResource, "pu_pd", "pd");

        /* Enable all the pins. */
        pinResource.used=true;
    }

    // TODO: should be PRU_ICSSG0_IEP0, PRU_ICSSG0_PRU0 by default
    // dividing view into 2 sections: IEP and PRU
    let peripheralRequirements = {
        name: interfaceName,
        displayName: interfaceName,
        interfaceName: interfaceName,
        resources: resources,       // push only the required resources
        /*
        filter: (peripheral) => {
            // The board is mapped to PRU0 pins
            let blocked_pins = ['PRU_ICSSG0_PRU1'];
            let found = blocked_pins.find(
                function(str) {
                    return str == peripheral.name;
                }
            );
            return !found;
        }, // works but does not hide further PRU pin options
        */
    };

    return peripheralRequirements;
}

function pinmuxRequirements(inst) {

    let iep = getPeripheralRequirements(inst, "IEP");
    let pru = getPeripheralRequirements(inst, "PRU");
    // let gpio = getPeripheralRequirements(inst, "GPIO");

    /* set default values for "rx" and "pu_pd" for different pins, based on use case */
    pinmux.setPeripheralPinConfigurableDefault(iep, "EDC_SYNC_OUT0", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault(iep, "EDC_SYNC_OUT0", "pu_pd", "nopull");
    pinmux.setPeripheralPinConfigurableDefault(iep, "EDC_SYNC_OUT1", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault(iep, "EDC_SYNC_OUT1", "pu_pd", "nopull");

    if (inst.interface === "byteParallel") {
        pinmux.setPeripheralPinConfigurableDefault(pru, "GPO8", "rx", false);
        pinmux.setPeripheralPinConfigurableDefault(pru, "GPO8", "pu_pd", "nopull");
        pinmux.setPeripheralPinConfigurableDefault(pru, "GPO9", "rx", false);
        pinmux.setPeripheralPinConfigurableDefault(pru, "GPO10", "rx", false);
        pinmux.setPeripheralPinConfigurableDefault(pru, "GPO11", "rx", false);
        pinmux.setPeripheralPinConfigurableDefault(pru, "GPO12", "rx", false);
        pinmux.setPeripheralPinConfigurableDefault(pru, "GPO13", "rx", false);
        pinmux.setPeripheralPinConfigurableDefault(pru, "GPO14", "rx", false);
        pinmux.setPeripheralPinConfigurableDefault(pru, "GPO15", "rx", false);
        pinmux.setPeripheralPinConfigurableDefault(pru, "GPO15", "pu_pd", "pu");
    }
    if (inst.interface === "serial") {
        pinmux.setPeripheralPinConfigurableDefault(pru, "GPO0", "rx", false);
        pinmux.setPeripheralPinConfigurableDefault(pru, "GPO1", "rx", false);
        pinmux.setPeripheralPinConfigurableDefault(pru, "GPO2", "rx", false);
        pinmux.setPeripheralPinConfigurableDefault(pru, "GPO3", "rx", false);
        pinmux.setPeripheralPinConfigurableDefault(pru, "GPO4", "rx", false);
        pinmux.setPeripheralPinConfigurableDefault(pru, "GPO5", "rx", false);
        pinmux.setPeripheralPinConfigurableDefault(pru, "GPO6", "rx", false);
        pinmux.setPeripheralPinConfigurableDefault(pru, "GPO9", "rx", false);
        pinmux.setPeripheralPinConfigurableDefault(pru, "GPO10", "rx", false);
        pinmux.setPeripheralPinConfigurableDefault(pru, "GPO11", "rx", false);
        pinmux.setPeripheralPinConfigurableDefault(pru, "GPO12", "rx", false);
        pinmux.setPeripheralPinConfigurableDefault(pru, "GPO13", "rx", false);
        pinmux.setPeripheralPinConfigurableDefault(pru, "GPO14", "rx", false);
        pinmux.setPeripheralPinConfigurableDefault(pru, "GPO15", "rx", false);
    }

    pinmux.setPeripheralPinConfigurableDefault(pru, "GPO18", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault(pru, "GPO18", "pu_pd", "nopull");
    pinmux.setPeripheralPinConfigurableDefault(pru, "GPO19", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault(pru, "GPO19", "pu_pd", "nopull");

    return [iep, pru, /*gpio */];
}

function getInterfaceNameList(inst) {

    return [
        getInterfaceName(inst, "IEP"),
        getInterfaceName(inst, "PRU"),
        // "GPIO"
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

function getOsrPinmuxSettings(instance) {
    switch (instance.overSampling) {
        case "osr2":
            return ["pu", "pd", "pd"];
        case "osr4":
            return ["pd", "pu", "pd"];
        case "osr8":
            return ["pu", "pu", "pd"];
        case "osr16":
            return ["pd", "pd", "pu"];
        case "osr32":
            return ["pu", "pd", "pu"];
        case "osr64":
            return ["pd", "pu", "pu"];
        case "disabled":
        default:
            return ["pd", "pd", "pd"];
    }
    /* ["pu", "pu", "pu"] is invalid input */
}

function moduleInstances(instance) {
    // if the pin is not being used or hardwired just unselect the pin
    let modInstances = [{
            name: "power1",
            displayName: "Adapter Power Enable Pin 1",
            moduleName: "/drivers/gpio/gpio",
            description: "DCDC_5V5_EN pin",
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
            name: "power2",
            displayName: "Adapter Power Enable Pin 2",
            moduleName: "/drivers/gpio/gpio",
            description: "EVM_5V0_EN pin",
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
            name: "power3",
            displayName: "Adapter Power Enable Pin 3",
            moduleName: "/drivers/gpio/gpio",
            description: "EVM_5V5_EN pin",
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
            name: "oversampling0",
            displayName: "OS0 Pin",
            moduleName: "/drivers/gpio/gpio",
            args: {
                GPIO: {
                    gpioPin: {
                        pu_pd: getOsrPinmuxSettings(instance)[0],
                    },
                },
            },
            requiredArgs: {
                pinDir: "OUTPUT",
                useMcuDomainPeripherals: false,
            },
        },
        {
            name: "oversampling1",
            displayName: "OS1 Pin",
            moduleName: "/drivers/gpio/gpio",
            args: {
                GPIO: {
                    gpioPin: {
                        pu_pd: getOsrPinmuxSettings(instance)[1],
                    },
                },
            },
            requiredArgs: {
                pinDir: "OUTPUT",
                useMcuDomainPeripherals: false,
            },
        },
        {
            name: "oversampling2",
            displayName: "OS2 Pin",
            moduleName: "/drivers/gpio/gpio",
            args: {
                GPIO: {
                    gpioPin: {
                        pu_pd: getOsrPinmuxSettings(instance)[2],
                    },
                },
            },
            requiredArgs: {
                pinDir: "OUTPUT",
                useMcuDomainPeripherals: false,
            },
        },
        {
            name: "range",
            displayName: "RANGE",
            moduleName: "/drivers/gpio/gpio",
            requiredArgs: (function(inst) {
                let pu_pd = "pd";
                if (inst.inputRange === "range5V") pu_pd = "pd";    // 5V
                else                               pu_pd = "pu";    // 10V
                return {
                    pinDir: "OUTPUT",
                    GPIO: {
                        gpioPin: {
                            pu_pd: pu_pd,
                        },
                    },
                    useMcuDomainPeripherals: false,
                };
            })(instance),
        },
        {
            name: "ref",
            displayName: "REFSEL",
            moduleName: "/drivers/gpio/gpio",
            requiredArgs: (function(inst) {
                let pu_pd = "pu";
                if (inst.reference) pu_pd = "pu";   // internal
                else                pu_pd = "pd";   // external
                return {
                    pinDir: "OUTPUT",
                    GPIO: {
                        gpioPin: {
                            pu_pd: pu_pd,
                        },
                    },
                    useMcuDomainPeripherals: false,
                };
            })(instance),
        },
        {
            name: "stby",
            displayName: "S̅T̅B̅Y̅",
            moduleName: "/drivers/gpio/gpio",
            requiredArgs: {
                pinDir: "OUTPUT",
                useMcuDomainPeripherals: false,
            },
            args: {
                GPIO: {
                    gpioPin: {
                        pu_pd: "pu",
                    },
                },
            },
        },
        {
            // jumper J9 on Adapter board: was meant for switching between serial and eSPI on 9224 adc
            // SELECTED INTERFACE MODE  | PAR/SER/BYTE SEL | DB15/BYTE SEL
            // Parallel interface       |       0          |       x
            // Parallel byte interface  |       1          |       1
            // Serial interface         |       1          |       0
            name: "par",
            displayName: " P̅A̅R̅/SER/BYTE SEL",
            moduleName: "/drivers/gpio/gpio",
            requiredArgs: (function(inst) {
                let pu_pd = "pu";
                if (inst.interface === "parallel")
                    pu_pd = "pd";
                else
                    pu_pd = "pu";
                return {
                    pinDir: "OUTPUT",
                    GPIO: {
                        gpioPin: {
                            pu_pd: pu_pd,
                        },
                    },
                    useMcuDomainPeripherals: false,
                };
            })(instance),
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
        minInstanceCount : 0,
        defaultInstanceCount: 0,
        args: {
            icssInstance: instance.icssInstance,
            noOfBuffers: parseInt(instance.channelsInUse),
            pruCore: "PRU0",
        },
    });

    return (modInstances);
}

let t_and_m_adapter_top_module_name = "/pru_io/adc/ads85x8/t_and_m_adapter";

let t_and_m_adapter_top_module = {
    displayName: "ADC Configuration",

    templates: {
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: t_and_m_adapter_top_module_name,
        },
    },
    defaultInstanceName: "CONFIG_ADS8598H_PINMUX",
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
            description: "Keep the jumper J9 on Adapter Board in parallel mode for all cases",
            options: [{
                name: "parallel",
                displayName: "Parallel (16Bits)",
            },
            {
                name: "byteParallel",
                displayName: "Byte Parallel (8Bits)",
            },
            {
                name: "serial",
                displayName: "Serial",
            }],
            default: "parallel",
            getDisabledOptions: () => {
                return [{
                    name: "serial",
                    reason: "The mode is not yet supported"
                }]
            },
        },
        {
            name: "overSampling",
            displayName: "Oversampling Mode",
            options: [{
                name: "disabled",
                displayName: "Disabled",
            },
            {
                name: "osr2",
                displayName: "OSR2",
            },
            {
                name: "osr4",
                displayName: "OSR4",
            },
            {
                name: "osr8",
                displayName: "OSR8",
            },
            {
                name: "osr16",
                displayName: "OSR16",
            },
            {
                name: "osr32",
                displayName: "OSR32",
            },
            {
                name: "osr64",
                displayName: "OSR64",
            }],
            default: "disabled",
        },
        {
            name: "inputRange",
            displayName: "Input Voltage Range",
            options: [{
                name: "range5V",
                displayName: "±5 V",
            },
            {
                name: "range10V",
                displayName: "±10 V",
            }],
            default: "range5V",
        },
        {
            name: "channelsInUse",
            displayName: "Channels In Use",
            description: "Number of adc channels to use for sampling from 1 to n",
            options: [
                { name: "1", },
                { name: "2", },
                { name: "3", },
                { name: "4", },
                { name: "5", },
                { name: "6", },
                { name: "7", },
                { name: "8", },
            ],
            default: "1",
        },
        {
            name: "samplingRate",
            displayName: "Sampling Rate",
            description: "Should be less than the max rate supported by selected adc",
            default: 40000,
        },
        {
            name: "reference",
            displayName: "Use Internal Reference Voltage",
            default: true,
        },
        {
            name: "combinedConvst",
            displayName: "Use Combined A & B CONVST Pin",
            description: "Adjust the jumper J7 on Adapter Board for this setting to work",
            default: true,
        },
    ],
    pinmuxRequirements,
    getInterfaceNameList,
    getPeripheralPinNames,
    moduleInstances,
    sharedModuleInstances,
    moduleInstances: moduleInstances,
};

function validate(inst, report) {
    common.validate.checkSameInstanceName(inst, report);
}

function sharedModuleInstances(instance) {
    let modInstances = [];

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

exports = t_and_m_adapter_top_module;
