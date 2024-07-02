let path = require('path');

let device = "am65x";

const files_r5f = {
    common: [
        "flash.c",
        "flash_nor_ospi.c",
        "led.c",
        "led_tpic2810.c",
        "nor_spi_sfdp.c",
    ],
};

const filedirs = {
    common: [
        "flash",
        "flash/sfdp",
        "flash/ospi",
        "led",
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
    if(buildOption.cpu.match(/r5f*/)) {
        build_property.files = files_r5f;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
