let path = require('path');

let device = "am263px";

const files_r5f = {
    common: [
        "eeprom.c",
        "eeprom_cat24m.c",
        "ethphy.c",
        "ethphy_dp83869.c",
        "ethphy_dp83826e.c",
        "flash.c",
        "flash_nor_ospi.c",
        "flash_nand_ospi.c",
        "led.c",
        "led_gpio.c",
        "led_tpic2810.c",
        "nor_spi_sfdp.c",
        "ioexp_tca6424.c",
        "ioexp_tca6416.c",
        "pmic_tps653860xx.c",
        "pmic.c",
        "pmic_common.c",
        "pmic_core.c",
        "pmic_io.c",
        "pmic_power.c",
        "pmic_wdg.c",
        "pmic_lld.c"
    ],
};

const files_m4f = {
    common: [

    ],
};

const filedirs = {
    common: [
        "eeprom",
        "ethphy",
        "flash",
        "flash/sfdp",
        "flash/ospi",
        "led",
        "ioexp",
        "pmic",
        "pmic/pmic_lld/src",
    ],
};

const includes = {
    common: [
        "pmic",
        "pmic/pmic_lld/src",
        "pmic/pmic_lld/include",
        "pmic/pmic_lld/include/cfg/tps65386x",
        "pmic/pmic_lld/src/cfg/tps65386x",
    ],
};

const buildOptionCombos = [
    { device: device, cpu: "r5f", cgt: "ti-arm-clang"},
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
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
