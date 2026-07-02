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
        "ncm_device.c",
        "no_os_gadget.c",

        "dhserver.c",
		"dnserver.c",

		/* LWIP SRC code */
		"altcp.c",
		"altcp_alloc.c",
		"altcp_tcp.c",
		"def.c",
		"dns.c",
		"inet_chksum.c",
		"init.c",
		"ip.c",
		"mem.c",
		"memp.c",
		"netif.c",
		"pbuf.c",
		"raw.c",
		"stats.c",
		"sys.c",
		"tcp.c",
		"tcp_in.c",
		"tcp_out.c",
		"timeouts.c",
		"udp.c",
		"autoip.c",
		"dhcp.c",
		"etharp.c",
		"icmp.c",
		"igmp.c",
		"ip4.c",
		"ip4_addr.c",
		"ip4_frag.c",
		"ethernet.c",
		"slipif.c",
		"httpd.c",
		"lwiperf.c",
		"fs.c",
		"err.c"
    ],
};

const filedirs = {
    common: [
        "../tinyusb/tinyusb-stack_0.20.0/src",
        "../tinyusb/tinyusb-stack_0.20.0/src/common",
        "../tinyusb/tinyusb-stack_0.20.0/src/device",
        "../tinyusb/tinyusb-stack_0.20.0/src/class/net",
        "../tinyusb/portable/am261x",
        "../synp/",
        "../synp/dwc3/",
        "../synp/include",
        "../synp/soc/",
        "../synp/soc/am261x",
        "../tinyusb/tinyusb-stack_0.20.0/lib/networking",

        "../../networking/lwip/lwip-stack/src/core",
		"../../networking/lwip/lwip-stack/src/api",
		"../../networking/lwip/lwip-stack/src/core/ipv4",
		"../../networking/lwip/lwip-stack/src/netif",
		"../../networking/lwip/lwip-stack/src/apps/http/",
		"../../networking/lwip/lwip-stack/src/apps/lwiperf/",
    ],
};

// Suppress warnings in this directories
const third_party_filedirs = {
    common: [
        "tinyusb-stack_0.20.0/",
    ]
}

const includes_common = {
    common: [
        "../../drivers/hw_include",
        "../../drivers/hw_include/am261x",
        "../../drivers/soc/am261x",
        "../../kernel/freertos/FreeRTOS-Kernel/include",
        "../../kernel/freertos/config/am261x/r5f",
        "../tinyusb/config/freertos/am261x/ncm_config",
        "../tinyusb/tinyusb-stack_0.20.0/src",
        "../tinyusb/tinyusb-stack_0.20.0/src/common",
        "../tinyusb/tinyusb-stack_0.20.0/src/device",
        "../tinyusb/tinyusb-stack_0.20.0/src/class/net",
        "../synp/",
        "../synp/dwc3/",
        "../synp/include",
        "../synp/soc/",
        "../synp/soc/am261x",
        "../tinyusb/tinyusb-stack_0.20.0/lib/networking",
        "../../networking/lwip/lwip-stack/src/include",
		"../../networking/lwip/lwip-stack/src/include/ipv4",
		"../../networking/lwip/lwip-stack/src/include/lwip/apps",
		"../../networking/lwip/lwip-stack/src/include/lwip",
		"../../networking/lwip/lwip-config/am261x/usb/",
		"../../networking/lwip/lwip-port/include/",
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
    release: [
    ],
};

const cflags_iar_arm = {
    common: [
        "--diag_suppress=Pe111"
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
    property.name = "usbd_tusb_ncm_freertos";
    property.isInternal = false;
    property.isSkipTopLevelBuild = false;
    property.buildOptionCombos = buildOptionCombos;
    property.tag = "ncm_freertos";
    /* Ignore this library for cpp build */
    property.ignore_cpp = true ;

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
    }
    else if(buildOption.cgt.match(/iar-arm*/)) {
        build_property.includes = includes_iar_arm;
        build_property.cflags = cflags_iar_arm;
        build_property.third_party_files = third_party_filedirs;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
