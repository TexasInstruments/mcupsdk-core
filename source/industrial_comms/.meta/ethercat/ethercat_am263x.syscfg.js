let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");

let icss_instances = [
    {
        name: "ICSSM0",
        displayName: "ICSSM0",
    },
];

function getIcssInstancesArr()
{
    return icss_instances;
}

function getRequiredArgsIcssInstance(instance)
{
    let arr = {};

    arr.instance = instance.instance;

    return arr;
}

function getInterfaceName(peripheralName)
{
    if(peripheralName !== "")
    {
        return `ICSSM_${peripheralName}`;
    }
    else
    {
        return "ICSSM";
    }
}

function getInterfacePinList(inst, peripheralName)
{
    let interfaceName = getInterfaceName(peripheralName);
    let pinList = [];

    if(peripheralName=="IEP")
    {
        pinList.push("PR0_IEP0_EDC_SYNC_OUT0");
        pinList.push("PR0_IEP0_EDC_SYNC_OUT1");
        pinList.push("PR0_IEP0_EDIO_DATA_IN_OUT30");
        pinList.push("PR0_IEP0_EDIO_DATA_IN_OUT31");
    }
    else
    {
        pinList.push("PR0_MDIO_MDIO");
        pinList.push("PR0_MDIO_MDC");
        pinList.push("PR0_PRU0_GPIO0");
        pinList.push("PR0_PRU0_GPIO1");
        pinList.push("PR0_PRU0_GPIO2");
        pinList.push("PR0_PRU0_GPIO3");
        pinList.push("PR0_PRU0_GPIO4");
        pinList.push("PR0_PRU0_GPIO5");
        pinList.push("PR0_PRU0_GPIO6");
        pinList.push("PR0_PRU0_GPIO8");
        pinList.push("PR0_PRU0_GPIO9");
        pinList.push("PR0_PRU0_GPIO10");
        pinList.push("PR0_PRU0_GPIO11");
        pinList.push("PR0_PRU0_GPIO12");
        pinList.push("PR0_PRU0_GPIO13");
        pinList.push("PR0_PRU0_GPIO14");
        pinList.push("PR0_PRU0_GPIO15");
        pinList.push("PR0_PRU0_GPIO16");
        pinList.push("PR0_PRU1_GPIO0");
        pinList.push("PR0_PRU1_GPIO1");
        pinList.push("PR0_PRU1_GPIO2");
        pinList.push("PR0_PRU1_GPIO3");
        pinList.push("PR0_PRU1_GPIO4");
        pinList.push("PR0_PRU1_GPIO5");
        pinList.push("PR0_PRU1_GPIO6");
        pinList.push("PR0_PRU1_GPIO8");
        pinList.push("PR0_PRU1_GPIO9");
        pinList.push("PR0_PRU1_GPIO10");
        pinList.push("PR0_PRU1_GPIO11");
        pinList.push("PR0_PRU1_GPIO12");
        pinList.push("PR0_PRU1_GPIO13");
        pinList.push("PR0_PRU1_GPIO14");
        pinList.push("PR0_PRU1_GPIO15");
        pinList.push("PR0_PRU1_GPIO16");
    }

    return pinList;
}

function getPeripheralRequirements(inst, peripheralName)
{
    let interfaceName = getInterfaceName(peripheralName);
    let pinList = getInterfacePinList(inst, peripheralName);
    let resources = [];
    let device = common.getDeviceName();

    for(let pin of pinList)
    {
        let pinResource = pinmux.getPinRequirements(interfaceName, pin);

        /* make all pins as "rx" and then override to make "rx" as false as needed  */
        pinmux.setConfigurableDefault( pinResource, "rx", true );

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

function getPinmuxRequirements(inst) {

    let icssm = getPeripheralRequirements(inst, "");
    let iep = getPeripheralRequirements(inst, "IEP");

    return [iep, icssm];

}

function getInterfaceNameList(inst) {

    return [
        getInterfaceName("IEP"),
        getInterfaceName(""),
    ];
}

function getPeripheralPinNames(inst)
{
    let pinList = [];

    pinList = pinList.concat( getInterfacePinList(inst, "IEP"),
                    getInterfacePinList(inst, ""),
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
