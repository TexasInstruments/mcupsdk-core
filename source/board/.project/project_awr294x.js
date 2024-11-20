let path = require('path');

let device = "awr294x";

const files_r5f = {
    common: [
        "eeprom.c",
        "eeprom_cav24c.c",
        "phy_common_priv.c",
        "dp83tc812.c",
        "dp83tg720.c",
        "dp83869.c",
        "dp83867.c",
        "dp83822.c",
        "dp83826.c",
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
        "ethphy/enet/rtos_drivers/src",
        "ethphy/enet/rtos_drivers/include",
        "flash",
        "flash/qspi",
        "flash/sfdp",
        "led",
    ],
};

const includes = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/board/ethphy/enet/rtos_drivers/include",
        "${MCU_PLUS_SDK_PATH}/source/board/ethphy/port",
        ],
}

const defines_r5f = {
    common: [
        "MCU_SDK_BUILD",
        "PHY_CFG_TRACE_LEVEL=3",
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
    build_property.includes = includes;
    if(buildOption.cpu.match(/r5f*/)) {
        build_property.files = files_r5f;
        build_property.defines = defines_r5f;
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
