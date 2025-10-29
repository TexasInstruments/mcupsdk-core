let path = require('path');

let device = "am261x";

const files = {
    common: [
        "usb_app_init.c",
        "device_wrapper.c",
        /* TinyUSB porting layer (usb/tinyusb/portable) */
        "dcd.c",
        /* TinyUSB core driver (usb/tinyusb/tinyusb-stack/src) */
        "tusb.c",
        "tusb_fifo.c",
        "usbd.c",
        "usbd_control.c",
        /* TinyUSB CDC class driver (usb/tinyusb/tinyusb-stack/class) */
        "cdc_device.c",
        "no_os_gadget.c",
    ],
};

const filedirs = {
    common: [
        "../tinyusb/tinyusb-stack/src",
        "../tinyusb/tinyusb-stack/src/common",
        "../tinyusb/tinyusb-stack/src/device",
        "../tinyusb/tinyusb-stack/src/class/cdc",
        "../tinyusb/portable/am261x",
        "../synp/",
        "../synp/include",
        "../synp/soc/",
        "../synp/soc/am261x",
        "../synp/dwc3/",
    ],
};

const includes_common = {
    common: [
        "../../drivers/hw_include",
        "../../drivers/hw_include/am261x",
        "../../drivers/soc/am261x",
        "../../kernel/freertos/FreeRTOS-Kernel/include",
        "../../kernel/freertos/config/am261x/r5f",
        "../tinyusb/config/freertos/am261x/cdc_config",
        "../tinyusb/tinyusb-stack/src",
        "../tinyusb/tinyusb-stack/src/common",
        "../tinyusb/tinyusb-stack/src/device",
        "../tinyusb/tinyusb-stack/src/class/cdc",
        "../synp/",
        "../synp/dwc3/",
        "../synp/include",
        "../synp/soc/",
        "../synp/soc/am261x",
    ],
};

const includes_ti_arm_clang = {
    common: [
        ...includes_common.common,
        "../../kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F",
    ],
};

const includes_iar_arm = {
    common: [
        ...includes_common.common,
        "../../kernel/freertos/portable/IAR_ARM/ARM_CR5F",
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
    { device: device, cpu: "r5f", cgt: "ti-arm-clang", os: "freertos"},
    { device: device, cpu: "r5f", cgt: "iar-arm", os: "freertos"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "../..");
    property.type = "library";
    property.name = "usbd_tusb_cdc_freertos";
    property.isInternal = false;
    property.isSkipTopLevelBuild = false;
    property.buildOptionCombos = buildOptionCombos;
    property.tag = "cdc_freertos";

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.defines = defines;
    if(buildOption.cgt.match(/ti-arm-clang*/)) {
        build_property.includes = includes_ti_arm_clang;
        build_property.cflags = cflags_ti_arm_clang;
    } else if(buildOption.cgt.match(/iar-arm*/)) {
        build_property.includes = includes_iar_arm;
        build_property.cflags = cflags_iar_arm;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
