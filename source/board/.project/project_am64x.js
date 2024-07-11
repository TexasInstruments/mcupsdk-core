let path = require('path');

let device = "am64x";

const files_r5f = {
    common: [
        "eeprom.c",
        "eeprom_at24c.c",
        "ethphy.c",
        "ethphy_dp83869.c",
        "ethphy_dp83826e.c",
        "flash.c",
        "flash_nand_gpmc.c",
        "flash_nor_ospi.c",
        "ram.c",
        "psram_gpmc.c",
        "led.c",
        "led_gpio.c",
        "led_tpic2810.c",
        "led_ioexp.c",
        "ioexp_tca6424.c",
        "nor_spi_sfdp.c",
    ],
};

const files_m4f = {
    common: [
        "eeprom.c",
        "eeprom_at24c.c",
        "flash.c",
        "flash_nor_ospi.c",
        "led.c",
        "led_gpio.c",
        "led_tpic2810.c",
        "led_ioexp.c",
        "ioexp_tca6424.c",
        "nor_spi_sfdp.c",
    ],
};

const files_a53 = {
    common: [
		"flash.c",
        "flash_nand_ospi.c",
        "flash_nor_ospi.c",
        "ram.c",
        "psram_gpmc.c",
        "led.c",
        "led_gpio.c",
        "led_tpic2810.c",
        "led_ioexp.c",
        "nor_spi_sfdp.c",
        "ioexp_tca6424.c",
    ]
}

const filedirs = {
    common: [
        "eeprom",
        "ethphy",
        "flash",
        "flash/gpmc",
        "flash/sfdp",
        "flash/ospi",
        "ram",
        "ram/gpmc",
        "led",
        "ioexp",
    ],
};

const cflags_a53 = {
    common: [
        "-Wno-unused-function",
        "-Wno-uninitialized"
    ]
}

const buildOptionCombos = [
    { device: device, cpu: "r5f", cgt: "ti-arm-clang"},
    { device: device, cpu: "r5f", cgt: "gcc-armv7"},
    { device: device, cpu: "m4f", cgt: "ti-arm-clang"},
    { device: device, cpu: "a53", cgt: "gcc-aarch64"},
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
    if(buildOption.cpu.match(/m4f*/)) {
        build_property.files = files_m4f;
    }
    if(buildOption.cpu.match(/a53*/)) {
        build_property.files = files_a53;
		build_property.cflags = cflags_a53;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
