let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");

let icss_instances = [
    {
        name: "ICSSG0",
        displayName: "ICSSG0",
    },
    {
        name: "ICSSG1",
        displayName: "ICSSG1",
    }
];

function getIcssInstancesArr()
{
    return icss_instances;
}

function getRequiredArgsIcssInstance(instance)
{
    let arr = {};

    arr.instance = instance.instance;
    arr.iepSyncMode = true;

    return arr;
}

function getInterfaceName(inst, peripheralName)
{
    return `PRU_${inst.instance}_${peripheralName}`;
}

function getInterfacePinList(inst, peripheralName)
{
    let interfaceName = getInterfaceName(inst, peripheralName);
    let pinList = [];

    if(peripheralName=="IEP")
    {
        pinList.push("EDC_LATCH_IN0");
        pinList.push("EDC_LATCH_IN1");
        pinList.push("EDC_SYNC_OUT0");
        pinList.push("EDC_SYNC_OUT1");
    }
    else
    {
        pinList = pinmux.getInterfacePinList(interfaceName);
    }

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

        if((pinResource.name !== "MII0_COL") &&
            (pinResource.name !== "MII0_CRS") &&
            (pinResource.name !== "MII1_COL") &&
            (pinResource.name !== "MII1_CRS"))
        {
            resources.push( pinResource );
        }
    }

    let peripheralRequirements = {
        name: interfaceName,
        displayName: interfaceName,
        interfaceName: interfaceName,
        resources: resources,
    };

    return peripheralRequirements;
}

function getPinmuxRequirements(inst) {

    let mdio = getPeripheralRequirements(inst, "MDIO");
    let iep = getPeripheralRequirements(inst, "IEP");
    let mii_g_rt = getPeripheralRequirements(inst, "MII_G_RT");

    /* set default values for "rx" for different pins, based on use case */
    pinmux.setPeripheralPinConfigurableDefault( mdio, "MDC", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault( iep, "EDC_SYNC_OUT0", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault( iep, "EDC_SYNC_OUT1", "rx", false);
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

    return [mdio, iep, mii_g_rt];

}

function getInterfaceNameList(inst) {

    return [
        getInterfaceName(inst, "MDIO"),
        getInterfaceName(inst, "IEP"),
        getInterfaceName(inst, "MII_G_RT"),
    ];
}

function getPeripheralPinNames(inst)
{
    let pinList = [];

    pinList = pinList.concat( getInterfacePinList(inst, "MDIO"),
                    getInterfacePinList(inst, "IEP"),
                    getInterfacePinList(inst, "MII_G_RT")
    );
    return pinList;
}

exports = {
    getIcssInstancesArr,
    getRequiredArgsIcssInstance,
    getPinmuxRequirements,
    getInterfaceNameList,
    getPeripheralPinNames,
};
