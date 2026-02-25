let path = require('path');

let device = "am263x";

const files_r5f = {
    common: [
        /* DAP module */
        "Dap.c",
        "Dap_Core.c",
        "Dap_Link.c",
    ],
};

const filedirs = {
    common: [
        "dap",
        "dap/dap_core",
        "dap/dap_link",
        "dap/dap_interface",
    ],
};

const includes = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source",
        "${MCU_PLUS_SDK_PATH}/source/ai/dap",
    ],
};

const defines_r5f = {
    common: [
    ],
};

const buildOptionCombos = [
    { device: device, cpu: "r5f", cgt: "ti-arm-clang"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "library";
    property.name = "ai";
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

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
