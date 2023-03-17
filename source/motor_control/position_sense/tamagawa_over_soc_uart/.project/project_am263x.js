let path = require('path');

let device = "am263x";

const files = {
    common: [
        "tamagawa_soc_uart_driv.c",
    ],
};


const filedirs = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/motor_control/position_sense/tamagawa_over_soc_uart/driver",
    ],
};

const includes = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/motor_control/position_sense/tamagawa_over_soc_uart/include",
    ],
};

const buildOptionCombos = [
    { device: device, cpu: "r5f", cgt: "ti-arm-clang"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "library";
    property.name = "motorcontrol_tamagawa_over_soc_uart";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.includes = includes;

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
