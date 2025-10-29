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
        "dwc_queue.c",
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
        "../synp/include/",
        "../synp/dwc3/",
        "../synp/soc/am261x",
        "../synp/soc/",
        "../synp",
        "../tinyusb/tinyusb-stack/src/",
        "../tinyusb/tinyusb-stack/src/",
        "../tinyusb/tinyusb-stack/src/common",
        "../tinyusb/tinyusb-stack/src/device",
    ],
};

const defines = {
    common: [
        "TINYUSB_INTEGRATION",
        "CFG_TUSB_OS=OPT_OS_NONE"
    ],
    debug: [
    ],
    release: [
    ],
};
const cflags_ti_arm_clang = {
    common: [
        "-Wno-address-of-packed-member",
    ],
};
const cflags_iar_arm = {
    common: [
    ],
};

const buildOptionCombos = [
    { device: device, cpu: "r5f", cgt: "ti-arm-clang", os: "nortos"},
    { device: device, cpu: "r5f", cgt: "iar-arm", os: "nortos"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "library";
    property.name = "usbd_synp_nortos";
    property.isInternal = false;
    property.isSkipTopLevelBuild = false;
    property.buildOptionCombos = buildOptionCombos;
    property.tag = "nortos";

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.includes = includes;
    build_property.defines = defines;
    if(buildOption.cgt.match(/ti-arm-clang*/)) {
        build_property.cflags = cflags_ti_arm_clang;
    }
    else if(buildOption.cgt.match(/iar-arm*/)) {
        build_property.cflags = cflags_iar_arm;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
