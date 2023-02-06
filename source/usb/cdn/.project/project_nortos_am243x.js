let path = require('path');

let device = "am243x";

const files = {
    common: [
        "cdn_osal_none.c",
        /* AM64x/AM243x porting layer (usb/cdn/soc)/am64x_am243x */
        "cdn_print.c",
        "cps.c",
        "usb_init.c",
        "usbss_functions.c",
        /* CDN device driver (usb/cdn/core_driver) */
        "cusb_ch9_sanity.c",
        "cusbd.c",
        "cusbd_obj_if.c",
        "cusbd_sanity.c",
        "cusbdma.c",
        "cusbdma_obj_if.c",
        "cusbdma_sanity.c",
        "list_sanity.c",
    ],
};

const filedirs = {
    common: [
        "../cdn/core_driver/common/src",
        "../cdn/core_driver/device/src",
        "../cdn/soc/am64x_am243x",
        "../cdn",
    ],
};

const includes = {
    common: [
        "../../drivers/hw_include",
        "../../drivers/hw_include/am64x_am243x",
        "../../kernel/dpl",
        "../../drivers/soc/am64x_am243x",
        "../cdn/core_driver/common/src",
        "../cdn/core_driver/common/include",
        "../cdn/core_driver/device/src",
        "../cdn/core_driver/device/include",
        "../cdn/include",
        "../cdn/soc/am64x_am243x",
        "../tinyusb/tinyusb-stack/src",
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
    property.name = "usbd_cdn_nortos";
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
    build_property.cflags = cflags;

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
