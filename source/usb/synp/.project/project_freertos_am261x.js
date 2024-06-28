let path = require('path');

let device = "am261x";

const files = {
    common: [
        /* dwc */
        "cil_intr.c",
        "cil.c",
        "ep0.c",
        "no_os_hiber.c",
        "pcd_hiber.c",
        "pcd_intr.c",
        "pcd.c",
    ],
};

const filedirs = {
    common: [
        "../synp/include/",
        "../synp/dwc3/",
        "../synp/soc/",
        "../synp",
    ],
};

const includes = {
    common: [
        "../../drivers/hw_include",
        "../../drivers/hw_include/am261x",
        "../../drivers/soc/am261x",
        "../../kernel/freertos/FreeRTOS-Kernel/include",
        "../../kernel/freertos/config/am261x/r5f",
        "../../kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F",
        "../synp/include/",
        "../synp/dwc3/",
        "../synp/soc/am261x",
        "../synp/soc/",
        "../synp",
        "../tinyusb/tinyusb-stack/src/",
        "../tinyusb/tinyusb-stack/src/common",
        "../tinyusb/tinyusb-stack/src/device",
    ],
};

const defines = {
    common: [
        "TINYUSB_INTEGRATION",
        "CFG_TUSB_OS=OPT_OS_FREERTOS"
    ],
    debug: [
    ],
    release: [
    ],
};

const cflags = {
    common: [
        "-Wno-address-of-packed-member",
    ],
};

const buildOptionCombos = [
    { device: device, cpu: "r5f", cgt: "ti-arm-clang"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "library";
    property.name = "usbd_synp_freertos";
    property.isInternal = false;
    property.isSkipTopLevelBuild = false;
    property.buildOptionCombos = buildOptionCombos;
    property.tag = "freertos";

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.includes = includes;
    build_property.defines = defines;
    build_property.cflags = cflags;

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
