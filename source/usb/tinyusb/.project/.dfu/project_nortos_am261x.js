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
        "dfu_device.c",
        "no_os_gadget.c",
    ],
};

const filedirs = {
    common: [
        "../tinyusb/tinyusb-stack_0.20.0/src",
        "../tinyusb/tinyusb-stack_0.20.0/src/common",
        "../tinyusb/tinyusb-stack_0.20.0/src/device",
        "../tinyusb/tinyusb-stack_0.20.0/src/class/dfu",
        "../tinyusb/portable/am261x",
        "../synp/",
        "../synp/dwc3/",
        "../synp/include",
        "../synp/soc/",
        "../synp/soc/am261x",
    ],
};

// Suppress warnings in this directories
const third_party_filedirs = {
    common: [
        "tinyusb-stack_0.20.0/",
    ]
}

const includes = {
    common: [
        "../../drivers/hw_include",
        "../../drivers/hw_include/am261x",
        "../../drivers/soc/am261x",
        "../tinyusb/config/nortos/am261x",
        "../tinyusb/tinyusb-stack_0.20.0/src",
        "../tinyusb/tinyusb-stack_0.20.0/src/common",
        "../tinyusb/tinyusb-stack_0.20.0/src/device",
        "../tinyusb/tinyusb-stack_0.20.0/src/class/dfu",
        "../tinyusb/config/nortos/am261x/dfu_config",
        "../synp/",
        "../synp/dwc3/",
        "../synp/include",
        "../synp/soc/",
        "../synp/soc/am261x",
    ],
};

const defines = {
    common: [
        "TINYUSB_INTEGRATION",
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

    property.dirPath = path.resolve(__dirname, "../..");
    property.type = "library";
    property.name = "usbd_tusb_dfu_nortos";
    property.isInternal = false;
    property.isSkipTopLevelBuild = false;
    property.buildOptionCombos = buildOptionCombos;
    property.tag = "dfu_nortos";
    property.ignore_cpp = true ;
    
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
        build_property.third_party_files = third_party_filedirs;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
