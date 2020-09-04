
let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");
let tamagawa_module_name = "/motor_control/position_sense/tamagawa";
let soc = system.getScript(`/drivers/pruicss/soc/pruicss_${common.getSocName()}`);

function getConfigArr() {
    return soc.getConfigArr();
}

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
    
    let uart = getPeripheralRequirements(inst, "UART");    

    /* set default values for "rx" for different pins, based on use case */
    pinmux.setPeripheralPinConfigurableDefault( uart, "CTSn", "rx", true);
    pinmux.setPeripheralPinConfigurableDefault( uart, "RTSn", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault( uart, "RXD", "rx", true);
    pinmux.setPeripheralPinConfigurableDefault( uart, "TXD", "rx", false);   

    return [uart];
}

function getInterfaceNameList(inst) {

    return [        
        getInterfaceName(inst, "UART"),
    ];
}

function getPeripheralPinNames(inst)
{
    let pinList = [];

    pinList = pinList.concat( getInterfacePinList(inst, "UART"),
    );
    return pinList;
}

let tamagawa_module = {

    displayName: "Tamagawa Position Encoder",
    templates: {
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: tamagawa_module_name,
        },
    },
    defaultInstanceName: "CONFIG_TAMAGAWA",
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
    moduleStatic: {
        modules: function(inst) {
            return [{
                name: "system_common",
                moduleName: "/system_common",
            }]
        },
    },
    pinmuxRequirements,
    getInterfaceNameList,
    getPeripheralPinNames,
    sharedModuleInstances: sharedModuleInstances,
};

function sharedModuleInstances(instance) {
    let modInstances = new Array();

    modInstances.push({
        name: "pru",
        displayName: "PRU ICSS Configuration",
        moduleName: '/drivers/pruicss/pruicss',
        requiredArgs: {
            instance: instance.instance,
            coreClk: 200*1000000,
        },
    });

    return (modInstances);
}

exports = tamagawa_module;
