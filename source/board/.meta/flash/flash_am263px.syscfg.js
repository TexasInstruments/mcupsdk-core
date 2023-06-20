let common = system.getScript("/common");

let supported_nor_spi_drivers = [
    {
        name: "ospi",
    },
];

let defaultFlashConfig = system.getScript("/board/flash/IS25LX256.json");

let defaultFlashConfigLP = system.getScript("/board/flash/IS25LX256.json");

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
    if(system.deviceData.device == "AM263Px") {
        return "IS25LX256";
    } else {
        return "IS25LX256";
    }
}

function getDefaultFlashConfig()
{
    if(system.deviceData.device == "AM263Px") {
        return defaultFlashConfig;
    } else {
        return defaultFlashConfigLP;
    }
}

function getDefaultProtocol()
{
    if(system.deviceData.device == "AM263Px") {
        return { name : "1s_1s_8s", displayName : "1S-1S-8D" };
    } else {
        return { name : "4s_4d_4d", displayName : "4S-4D-4D" };
    }
}

function getDefaultProtocolJson()
{
    if(system.deviceData.device == "AM263Px") {
        return "p118";
    } else {
        return "p444d";
    }
}

exports = {
    getDriverOptions,
    getDefaultDriver,
    getDefaultFlashName,
    getDefaultProtocol,
    getDefaultProtocolJson,
    getDefaultFlashConfig,
};
