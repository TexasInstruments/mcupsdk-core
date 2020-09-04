let path = require('path');

let device = "am273x";

const files_r5f = {
    common: [
        "eeprom.c",
        "eeprom_cav24c.c",
        "led.c",
        "led_gpio.c",
        "flash.c",
        "flash_nor_qspi.c",
        "nor_spi_sfdp.c",
    ],
};

const files_c66 = {
    common: [
        "eeprom.c",
        "eeprom_cav24c.c",
        "led.c",
        "led_gpio.c",
    ],
};

const filedirs = {
    common: [
        "eeprom",
        "flash",
        "flash/sfdp",
        "flash/qspi",
        "led",
    ],
};

const buildOptionCombos = [
    { device: device, cpu: "r5f", cgt: "ti-arm-clang"},
    { device: device, cpu: "c66", cgt: "ti-c6000"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "library";
    property.name = "board";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.filedirs = filedirs;
    if(buildOption.cpu.match(/r5f*/)) {
        build_property.files = files_r5f;
    }
    else
    {
        build_property.files = files_c66;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
