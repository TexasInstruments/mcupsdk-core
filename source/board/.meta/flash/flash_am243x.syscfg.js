let common = system.getScript("/common");

let supported_nor_spi_drivers = [
    {
        name: "ospi",
    },
];

let defaultFlashConfig = system.getScript("/board/flash/S28HS512T.json");

let defaultFlashConfigLP = system.getScript("/board/flash/S25HL512T.json");

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
    if(system.deviceData.device == "AM243x_ALV_beta") {
        return "S28HS512T";
    } else {
        return "S25HL512T";
    }
}

function getDefaultFlashConfig()
{
    if(system.deviceData.device == "AM243x_ALV_beta") {
        return defaultFlashConfig;
    } else {
        return defaultFlashConfigLP;
    }
}

function getDefaultProtocol()
{
    if(system.deviceData.device == "AM243x_ALV_beta") {
        return { name : "8d_8d_8d", displayName : "8D-8D-8D" };
    } else {
        return { name : "4s_4d_4d", displayName : "4S-4D-4D" };
    }
}

function getDefaultProtocolJson()
{
    if(system.deviceData.device == "AM243x_ALV_beta") {
        return "p888d";
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
