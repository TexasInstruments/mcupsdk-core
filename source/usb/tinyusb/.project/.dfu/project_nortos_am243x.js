let path = require('path');

let device = "am243x";

const files = {
    common: [
		/* cdn -> tusb  wrapper */
        "usb_wrapper.c",
        /* TinyUSB porting layer (usb/tinyusb/portable) */
        "dcd.c",
        /* TinyUSB core driver (usb/tinyusb/tinyusb-stack/src) */
        "tusb.c",
        "tusb_fifo.c",
        "usbd.c",
        "usbd_control.c",
        /* TinyUSB CDC class driver (usb/tinyusb/tinyusb-stack/class) */
        "dfu_device.c",
    ],
};

const filedirs = {
    common: [
        "../tinyusb/tinyusb-stack/src",
        "../tinyusb/tinyusb-stack/src/common",
        "../tinyusb/tinyusb-stack/src/device",
        "../tinyusb/tinyusb-stack/src/class/dfu",
        "../tinyusb/portable/am64x_am243x",
        "../cdn/include",
        "../cdn/soc/am64x_am243x",
    ],
};

const includes = {
    common: [
        "../../drivers/hw_include",
        "../../drivers/hw_include/am64x_am243x",
        "../../drivers/soc/am64x_am243x",
        "../tinyusb/config/nortos/am64x_am243x",
        "../tinyusb/tinyusb-stack/src",
        "../tinyusb/tinyusb-stack/src/common",
        "../tinyusb/tinyusb-stack/src/device",
        "../tinyusb/tinyusb-stack/src/class/dfu",
        "../tinyusb/config/nortos/am64x_am243x/dfu_config",
        "../cdn/core_driver/common/src",
        "../cdn/core_driver/common/include",
        "../cdn/core_driver/device/src",
        "../cdn/core_driver/device/include",
        "../cdn/include",
        "../cdn/soc/am64x_am243x",
    ],
};

const defines = {
    common: [
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
    { device: device, cpu: "r5f", cgt: "gcc-armv7"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "../..");
    property.type = "library";
    property.name = "usbd_tusb_dfu_nortos";
    property.isInternal = false;
    property.isSkipTopLevelBuild = false;
    property.buildOptionCombos = buildOptionCombos;
    property.tag = "dfu_nortos";

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.includes = includes;
    build_property.defines = defines;
    if(buildOption.cgt.match(/ti-arm-clang*/)) {
        build_property.cflags = cflags;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
