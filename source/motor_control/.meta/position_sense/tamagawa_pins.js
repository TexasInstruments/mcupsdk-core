
let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");


function getInterfaceName(inst)
{
    return `PRU_${inst.instance}_PRU`;
}

function getInterfacePinList(inst)
{
    let pinList = [];

    pinList.push({ pinName: "GPO2", displayName: "TAMAGAWA_CHANNEL0_TX_ENABLE", rx: false});
    pinList.push({ pinName: "GPO1", displayName: "TAMAGAWA_CHANNEL0_TX", rx: false});
    pinList.push({ pinName: "GPO0", displayName: "TAMAGAWA_CHANNEL0_CLK", rx: false});
    pinList.push({ pinName: "GPI13", displayName: "TAMAGAWA_CHANNEL0_RX", rx: true});

    pinList.push({ pinName: "GPO5", displayName: "TAMAGAWA_CHANNEL1_TX_ENABLE", rx: false});
    pinList.push({ pinName: "GPO4", displayName: "TAMAGAWA_CHANNEL1_TX", rx: false});
    pinList.push({ pinName: "GPO3", displayName: "TAMAGAWA_CHANNEL1_CLK", rx: false});
    pinList.push({ pinName: "GPI14", displayName: "TAMAGAWA_CHANNEL1_RX", rx: true});

    pinList.push({ pinName: "GPO8", displayName: "TAMAGAWA_CHANNEL2_TX_ENABLE", rx: false});
    pinList.push({ pinName: "GPO12", displayName: "TAMAGAWA_CHANNEL2_TX", rx: false});
    pinList.push({ pinName: "GPO6", displayName: "TAMAGAWA_CHANNEL2_CLK", rx: false});
    pinList.push({ pinName: "GPI11", displayName: "TAMAGAWA_CHANNEL2_RX", rx: true});

    return pinList;
}

function pinmuxRequirements(inst) {

    let interfaceName = getInterfaceName(inst);
    let pinList = getInterfacePinList(inst);
    let resources = [];

    for(let pin of pinList)
    {
        let pinResource = pinmux.getPinRequirements(interfaceName, pin.pinName, pin.displayName);

        pinmux.setConfigurableDefault( pinResource, "rx", pin.rx );

        resources.push( pinResource );
    }

    let peripheralRequirements = {
        name: interfaceName,
        displayName: interfaceName,
        interfaceName: interfaceName,
        resources: resources,
    };

    return [peripheralRequirements];
}

function getPeripheralPinNames(inst)
{
    let pinList = [];
    let pinNameList = [];

    pinList = getInterfacePinList(inst);

    for(let pin of pinList)
    {
        pinNameList.push( pin.pinName );
    }

    return pinNameList;
}


exports = {

    pinmuxRequirements,
    getInterfaceName,
    getPeripheralPinNames,
};
