
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
    if (common.getSocName() === "am64x")
    {
    let interfaceName = getInterfaceName(inst, peripheralName);
    let resources = [];
    let pinResource = {};

    if (name == "RGMII")
    {
    if (inst.enableRgmii1 == true)
    {
        pinResource = pinmux.getPinRequirements(interfaceName, "RGMII1_RD0", "RGMII1_RD0");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RGMII1_RD1", "RGMII1_RD1");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RGMII1_RD2", "RD2");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RGMII1_RD3", "RD3");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RGMII1_RX_CTL", "RX_CTL");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RGMII1_RXC", "RX_RXC");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RGMII1_TD0", "TD0");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RGMII1_TD1", "TD1");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RGMII1_TD2", "TD2");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RGMII1_TD3", "TD3");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RGMII1_TX_CTL", "TX_CTL");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RGMII1_TXC", "TX_RXC");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
    }

    if (inst.enableRgmii2 == true)
    {
        pinResource = pinmux.getPinRequirements(interfaceName, "RGMII2_RD0", "RGMII2_RD0");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RGMII2_RD1", "RD1");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RGMII2_RD2", "RD2");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RGMII2_RD3", "RD3");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RGMII2_RX_CTL", "RX_CTL");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RGMII2_RXC", "RX_RXC");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RGMII2_TD0", "TD0");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RGMII2_TD1", "TD1");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RGMII2_TD2", "TD2");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RGMII2_TD3", "TD3");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RGMII2_TX_CTL", "TX_CTL");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RGMII2_TXC", "TX_RXC");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
    }
    }
    else if (name == "RMII")
    {
        pinResource = pinmux.getPinRequirements(interfaceName, "RMII1_RXD0", "RXD0");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RMII1_RXD1", "RXD1");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RMII1_RX_ER", "RX_ER");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RMII1_TXD0", "TXD0");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RMII1_TXD1", "TXD1");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RMII1_CRS_DV", "CRS_DV");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);

        pinResource = pinmux.getPinRequirements(interfaceName, "RMII2_RXD0", "RXD0");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RMII2_RXD1", "RXD1");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RMII2_RX_ER", "RX_ER");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RMII2_TXD0", "TXD0");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RMII2_TXD1", "TXD1");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RMII2_CRS_DV", "CRS_DV");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
    }
    else if (name == "CPSW_CPTS")
    {
       pinResource = pinmux.getPinRequirements(interfaceName, "TS_SYNC", "TS_SYNC");
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
    } //  if (common.getSocName() === "am64x")
    else
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
    else if(peripheralName == "RMII1")
    {
        pinResource = pinmux.getPinRequirements(interfaceName, "RMII1_RXD0", "RXD0");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RMII1_RXD1", "RXD1");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RMII1_RX_ER", "RX_ER");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RMII1_TXD0", "TXD0");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RMII1_TXD1", "TXD1");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RMII1_CRS_DV", "CRS_DV");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
    }
	else if(peripheralName == "RMII2")
    {
        pinResource = pinmux.getPinRequirements(interfaceName, "RMII2_RXD0", "RXD0");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RMII2_RXD1", "RXD1");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RMII2_RX_ER", "RX_ER");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RMII2_TXD0", "TXD0");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RMII2_TXD1", "TXD1");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RMII2_CRS_DV", "CRS_DV");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
    }
    else if (name == "CPSW_CPTS")
    {
        pinResource = pinmux.getPinRequirements(interfaceName, "TS_SYNC", "TS_SYNC");
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
        let cptsTsSync = getPeripheralRequirements(inst, "CPSW3G", "CPSW_CPTS");
        pinmux.setPeripheralPinConfigurableDefault( cptsTsSync, "CPSW3G", "rx", false);
        perRequirements.push(cptsTsSync);
    }

    let mdio = getPeripheralRequirements(inst, "MDIO", "MDIO");

    /* set default values for "rx" for different pins, based on use case */
    pinmux.setPeripheralPinConfigurableDefault( mdio, "MDC", "rx", false);
    perRequirements.push(mdio);

    if (inst.phyToMacInterfaceMode === "RMII")
    {
        if (common.getSocName() === "am64x")
        {
            let rmii = getPeripheralRequirements(inst, "CPSW", "RMII");
            pinmux.setPeripheralPinConfigurableDefault( rmii, "RMII0_TXD0", "rx", false);
            pinmux.setPeripheralPinConfigurableDefault( rmii, "RMII0_TXD1", "rx", false);
            pinmux.setPeripheralPinConfigurableDefault( rmii, "RMII0_TXEN", "rx", false);
            pinmux.setPeripheralPinConfigurableDefault( rmii, "RMII1_TXD0", "rx", false);
            pinmux.setPeripheralPinConfigurableDefault( rmii, "RMII1_TXD1", "rx", false);
            pinmux.setPeripheralPinConfigurableDefault( rmii, "RMII1_TXEN", "rx", false);
            perRequirements.push(rmii);
        }
        else
        {
            let rmii = getPeripheralRequirements(inst, "RMII", "RMII");

            pinmux.setPeripheralPinConfigurableDefault( rmii, "RMII0_TXD0", "rx", false);
            pinmux.setPeripheralPinConfigurableDefault( rmii, "RMII0_TXD1", "rx", false);
            pinmux.setPeripheralPinConfigurableDefault( rmii, "RMII0_TXEN", "rx", false);
            pinmux.setPeripheralPinConfigurableDefault( rmii, "RMII1_TXD0", "rx", false);
            pinmux.setPeripheralPinConfigurableDefault( rmii, "RMII1_TXD1", "rx", false);
            pinmux.setPeripheralPinConfigurableDefault( rmii, "RMII1_TXEN", "rx", false);
            perRequirements.push(rmii);
        }
    }
    else
    {
        if (common.getSocName() === "am64x")
        {
            let rgmii1 = getPeripheralRequirements(inst, "CPSW", "RGMII");
            perRequirements.push(rgmii1);
        }
        else
        {
            if (inst.enableRgmii1 == true)
            {
                let rgmii1 = getPeripheralRequirements(inst, "RGMII", "RGMII1");
                perRequirements.push(rgmii1);
            }
            if (inst.enableRgmii2 == true)
            {
                let rgmii2 = getPeripheralRequirements(inst, "RGMII", "RGMII2");
                perRequirements.push(rgmii2);
            }
        }
    }
    return perRequirements;
}

function getInterfaceNameList(inst) {
    let interfaceNameList = []
    if (inst.enableTsOut === true)
    {
                interfaceNameList.push("CPSW_CPTS")
    }
    interfaceNameList.push(getInterfaceName(inst, "MDIO"));
    if (inst.phyToMacInterfaceMode === "RMII")
    {
        interfaceNameList.push(getInterfaceName(inst, "RMII"));
    }
    else
    {
        if (common.getSocName() === "am64x")
        {
            interfaceNameList.push(getInterfaceName(inst, "RGMII"));
        }
        else
        {
            if (inst.enableRgmii1 == true)
            {
            interfaceNameList.push(getInterfaceName(inst, "RGMII1"));
            }
            if (inst.enableRgmii2 == true)
            {
            interfaceNameList.push(getInterfaceName(inst, "RGMII2"));
            }
        }
    }

    return interfaceNameList;
}

function getPeripheralPinNames(inst)
{
    let pinList = [];

     if (inst.enableTsOut === true)
     {
         pinList = pinList.concat("TS_SYNC");
     }
     
    if (common.getSocName() != "am64x")
    {
        if (inst.phyToMacInterfaceMode === "RMII")
        {
            pinList = pinList.concat(getInterfacePinList(inst, "MDIO"),
                            getInterfacePinList(inst, "RMII"));
        }
        else
        {
            pinList = pinList.concat(getInterfacePinList(inst, "MDIO"),
                            getInterfacePinList(inst, "RGMII"));
    
        }
    }
    else
    {
        let cpswPinList =  getInterfacePinList(inst, "CPSW");
        let xmiipinList = [];
        if (inst.phyToMacInterfaceMode === "RMII")
        {
            for (let pin of cpswPinList)
            {
                if (pin.startsWith("RMII"))
                {
                    xmiipinList.push(pin);
                }
            }
        }
        else
        {
            for (let pin of cpswPinList)
            {;
                if (pin.startsWith("RGMII"))
                {
                    xmiipinList.push(pin);
                }
            }
        }
        pinList = pinList.concat(getInterfacePinList(inst, "MDIO"), xmiipinList);
    }
    return pinList;
}

function setNumPorts(inst, numPorts)
{
    inst.numPorts = numPorts;
}

let enet_cpsw_pinmux_module = {
    displayName: "CPSW pinmux config",
    longDescription: `This configures CPSW module pinmux`,
    alwaysShowLongDescription: false,
    defaultInstanceName: "ENET_CPSW_PINMUX",
    config: [
        {
            name: "phyToMacInterfaceMode",
            displayName: "RMII/RGMII",
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
            name: "numPorts",
            displayName: "Num Of Ports",
            default: 2,
            displayFormat: "dec",
            hidden: true,
            isInteger:true,
            range: [0, 2],
        },
        {
            name: "enableTsOut",
            displayName: "Enable CPTS TS Output",
            default: false,
            hidden: false,
        },
        {
            name: "enableRgmii1",
            displayName: "Enable RGMII1 for CPSW",
            default: true,
            hidden: false,
        },
        {
            name: "enableRgmii2",
            displayName: "Enable RGMII2 for CPSW",
            default: true,
            hidden: false,
        },
    ],
    getInstanceConfig,
    pinmuxRequirements,
    getInterfaceNameList,
    getPeripheralPinNames,
    setNumPorts,

};


exports = enet_cpsw_pinmux_module;
