let common = system.getScript("/common");

let supported_nor_spi_drivers = [
    {
        name: "ospi",
    },
];

let defaultFlashConfig = system.getScript("/board/flash/IS25LX256.json");

let defaultFlashConfigLP = system.getScript("/board/flash/IS25LX256.json");

let defaultNandFlashConfig = system.getScript("/board/flash/W25N01GVZEJ.json");

function getDriverOptions()
{
    return supported_nor_spi_drivers;
}

function getDefaultDriver()
{
    return supported_nor_spi_drivers[0].name;
}

function getDefaultFlashName()
{
    if(system.deviceData.device == "AM261x") {
        return "IS25LX256";
    } else {
        return "IS25LX256";
    }
}

function getDefaultFlashConfig()
{
    if(system.deviceData.device == "AM261x") {
        if(system.deviceData.package == "ZCZ_F")
        {
            defaultFlashConfig = system.getScript("/board/flash/IS25LX064.json");
        }
        return defaultFlashConfig;
    } else {
        return defaultFlashConfigLP;
    }
}

function getDefaultProtocol()
{
    if(system.deviceData.device == "AM261x") {
        return { name : "1s_1s_8s", displayName : "1S-1S-8S" };
    } else {
        return { name : "4s_4d_4d", displayName : "4S-4D-4D" };
    }
}

function getDefaultProtocolJson()
{
    if(system.deviceData.device == "AM261x") {
        return "p118";
    } else {
        return "p444d";
    }
}

function getDefaultNandProtocolJson()
{
    if(system.deviceData.device == "AM261x") {
        return "p114";
    } else {
        return "p444d";
    }
}

function getDefaultNandFlashName()
{
    return "W25N01GVZEJ";
}

function getDefaultNandProtocol()
{
    return { name : "1s_1s_4s", displayName : "1S-1S-4S" };
}

function getDefaultNandFlashConfig()
{
    return defaultNandFlashConfig;
}

exports = {
    getDriverOptions,
    getDefaultDriver,
    getDefaultFlashName,
    getDefaultProtocol,
    getDefaultProtocolJson,
    getDefaultNandProtocolJson,
    getDefaultFlashConfig,
    getDefaultNandFlashName,
    getDefaultNandProtocol,
    getDefaultNandFlashConfig,
};
