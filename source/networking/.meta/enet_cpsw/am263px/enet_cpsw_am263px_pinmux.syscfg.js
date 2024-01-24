
let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");
let device = common.getDeviceName();
let soc = system.getScript(`/networking/soc/networking_${common.getSocName()}`);

function getInstanceConfig(moduleInstance) {
    return {
        ...moduleInstance,
    };
};

function getPeripheralRequirements(inst, peripheralName, name)
{
    let interfaceName = getInterfaceName(inst, peripheralName);
    let resources = [];
    let pinResource = {};

    if(peripheralName == "RGMII")
    {
        pinResource = pinmux.getPinRequirements(interfaceName, "RD0", "RD0");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RD1", "RD1");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RD2", "RD2");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RD3", "RD3");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RX_CTL", "RX_CTL");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RXC", "RX_RXC");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "TD0", "TD0");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "TD1", "TD1");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "TD2", "TD2");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "TD3", "TD3");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "TX_CTL", "TX_CTL");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "TXC", "TX_RXC");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
    }
    else if(peripheralName == "RMII")
    {
        pinResource = pinmux.getPinRequirements(interfaceName, "RXD0", "RXD0");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RXD1", "RXD1");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RX_ER", "RX_ER");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "TXD0", "TXD0");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "TXD1", "TXD1");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "CRS_DV", "CRS_DV");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
    }
    else if (name == "CPSW_CPTS")
    {
        pinResource = pinmux.getPinRequirements(interfaceName, "CPTS0_TS_SYNC", "CPTS0_TS_SYNC");
        pinmux.setConfigurableDefault( pinResource, "rx", false );
        resources.push( pinResource);
    }
    else
    {
        let pinList = getInterfacePinList(inst, interfaceName);
        for(let pin of pinList)
        {
            pinResource = pinmux.getPinRequirements(interfaceName, pin);

            /* make all pins as "rx" and then override to make "rx" as false as needed  */
            pinmux.setConfigurableDefault( pinResource, "rx", true );

            resources.push( pinResource );
        }
    }

    let peripheralRequirements = {
        name: name,
        displayName: name,
        interfaceName: interfaceName,
        resources: resources,
    };

    return peripheralRequirements;
}

function getInterfaceName(inst, peripheralName)
{
    return `${peripheralName}`;
}

function getInterfacePinList(inst, peripheralName)
{
    let interfaceName = getInterfaceName(inst, peripheralName);
    let pinList = [];

    pinList = pinmux.getInterfacePinList(interfaceName);

    return pinList;
}

function pinmuxRequirements(inst) {

    let perRequirements = [];

    if (inst.enableTsOut === true)
    {
        let cptsTsSync = getPeripheralRequirements(inst, "CPTS", "CPSW_CPTS");
        pinmux.setPeripheralPinConfigurableDefault( cptsTsSync, "CPSW_CPTS", "rx", false);
        perRequirements.push(cptsTsSync);
    }

    let mdio = getPeripheralRequirements(inst, "MDIO", "MDIO");

    /* set default values for "rx" for different pins, based on use case */
    pinmux.setPeripheralPinConfigurableDefault( mdio, "MDC", "rx", false);
    perRequirements.push(mdio);

    if( inst.phyToMacInterfaceMode === "MII")
    {
        let mii1 = getPeripheralRequirements(inst, "MII", "MII1");
        let mii2 = getPeripheralRequirements(inst, "MII", "MII2");

        return [mdio, mii1, mii2];
    }
    else if( inst.phyToMacInterfaceMode === "RMII")
    {
        let rmii = getPeripheralRequirements(inst, "RMII", "RMII");

        pinmux.setPeripheralPinConfigurableDefault( rmii, "RMII1_TXD0", "rx", false);
        pinmux.setPeripheralPinConfigurableDefault( rmii, "RMII1_TXD1", "rx", false);
        pinmux.setPeripheralPinConfigurableDefault( rmii, "RMII1_TX_EN", "rx", false);
        pinmux.setPeripheralPinConfigurableDefault( rmii, "RMII2_TXD0", "rx", false);
        pinmux.setPeripheralPinConfigurableDefault( rmii, "RMII2_TXD1", "rx", false);
        pinmux.setPeripheralPinConfigurableDefault( rmii, "RMII2_TX_EN", "rx", false);

        perRequirements.push(rmii);
    }
    else
    {
        let rgmii1 = getPeripheralRequirements(inst, "RGMII", "RGMII1");
        let rgmii2 = getPeripheralRequirements(inst, "RGMII", "RGMII2");

        perRequirements.push(rgmii1);
        perRequirements.push(rgmii2);
    }
        return perRequirements;
}

function getInterfaceNameList(inst) {
    let interfaceNameList = []
    interfaceNameList.push(getInterfaceName(inst, "MDIO"))

    if (inst.enableTsOut === true)
    {
        interfaceNameList.push("CPSW_CPTS")
    }
    if (inst.phyToMacInterfaceMode === "MII")
    {
        interfaceNameList.push(getInterfaceName(inst, "MII1"));
        interfaceNameList.push(getInterfaceName(inst, "MII2"));
    }
    else if (inst.phyToMacInterfaceMode === "RMII")
    {
        interfaceNameList.push(getInterfaceName(inst, "RMII"));
    }
    else
    {
        interfaceNameList.push(getInterfaceName(inst, "RGMII1"));
        interfaceNameList.push(getInterfaceName(inst, "RGMII2"));
    }
    return interfaceNameList;
}

function getPeripheralPinNames(inst)
{
    let pinList = [];
    if (inst.enableTsOut === true)
    {
      pinList = pinList.concat( "CPTS0_TS_SYNC");
    }

    if(inst.phyToMacInterfaceMode === "MII")
    {
        pinList = pinList.concat( getInterfacePinList(inst, "MDIO"),
                        getInterfacePinList(inst, "MII" )
        );
    }
    else if(inst.phyToMacInterfaceMode === "RMII")
    {
        pinList = pinList.concat(getInterfacePinList(inst, "MDIO"),
                        getInterfacePinList(inst, "RMII" ));
    }
    else
    {
        pinList = pinList.concat( getInterfacePinList(inst, "MDIO"),
                        getInterfacePinList(inst, "RGMII" ));
    }
    return pinList;
}


let enet_cpsw_pinmux_module = {
    displayName: "CPSW pinmux config",
    longDescription: `This configures CPSW module pinmux`,
    alwaysShowLongDescription: false,
    defaultInstanceName: "ENET_CPSW_PINMUX",
    config: [
        {
            name: "phyToMacInterfaceMode",
            displayName: "RMII/RGMII/MII",
            default: "RGMII",
            options: [
                {
                    name: "RMII",
                },
                {
                    name: "RGMII",
                },
            ],
        },
        {
            name: "enableTsOut",
            displayName: "Enable CPTS TS Output",
            default: false,
            hidden: false,
        },
    ],
    getInstanceConfig,
    pinmuxRequirements,
    getInterfaceNameList,
    getPeripheralPinNames,

};


exports = enet_cpsw_pinmux_module;
