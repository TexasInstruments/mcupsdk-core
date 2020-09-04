

let common = system.getScript("/common");

let ethphy_devices = [
    {
        name: "DP83869",
    },
    {
        name: "NONE",
        description: "Use this option to disable ETHPHY driver code generation"
    },
    {
        name: "CUSTOM",
        description: "Use this option to specify a custom ETHPHY device name using the text box below"
    }
];

let mdio_instances = [
    {
        name: "ICSSG0",
        displayName: "ICSSG0 MDIO",
    },
    {
        name: "ICSSG1",
        displayName: "ICSSG1 MDIO",
    }
];

function getConfigArr()
{
    return ethphy_devices;
}

function getMdioInstancesArr()
{
    return mdio_instances;
}

exports = {
    getConfigArr,
    getMdioInstancesArr,
};
