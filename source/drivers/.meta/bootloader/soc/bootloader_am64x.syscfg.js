let common = system.getScript("/common");

const bootloader_config_r5fss = [
    {
        name            : "BOOTLOADER0",
    },
];

const bootloader_bootmedia = [

    { name: "FLASH", displayName: "Flash" },
    { name: "MEM", displayName: "SOC Memory" },
    { name: "EMMC", displayName: "EMMC"},
    { name: "BUFIO", displayName: "Buffered IO Device (UART)"},
];

function getDefaultConfig()
{
    return bootloader_config_r5fss[0];
}

function getConfigArr() {

    return bootloader_config_r5fss;
}

function getBootMediaArr() {

	return bootloader_bootmedia;
}

exports = {
    getDefaultConfig,
    getConfigArr,
    getBootMediaArr,
};

