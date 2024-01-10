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
        /* TinyUSB NCM class driver (usb/tinyusb/tinyusb-stack/class) */
        "ncm_device.c",
		/* TinyUSB networking lib */
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
        "../tinyusb/tinyusb-stack/src",
        "../tinyusb/tinyusb-stack/lib/networking",
        "../tinyusb/tinyusb-stack/src/common",
        "../tinyusb/tinyusb-stack/src/device",
        "../tinyusb/tinyusb-stack/src/class/net",
        "../tinyusb/portable/am64x_am243x",
        "../cdn/include",
        "../cdn/soc/am64x_am243x",
		"../../networking/lwip/lwip-stack/src/core",
		"../../networking/lwip/lwip-stack/src/api",
		"../../networking/lwip/lwip-stack/src/core/ipv4",
		"../../networking/lwip/lwip-stack/src/netif",
		"../../networking/lwip/lwip-stack/src/apps/http/",
		"../../networking/lwip/lwip-stack/src/apps/lwiperf/",
    ],
};

const includes = {
    common: [
        "../../drivers/hw_include",
        "../../drivers/hw_include/am64x_am243x",
        "../../drivers/soc/am64x_am243x",
        "../../kernel/freertos/FreeRTOS-Kernel/include",
        "../../kernel/freertos/config/am243x/r5f",
        "../../kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F",
        "../tinyusb/tinyusb-stack/src",
        "../tinyusb/tinyusb-stack/src/common",
        "../tinyusb/tinyusb-stack/src/device",
        "../tinyusb/tinyusb-stack/src/class/net",
        "../tinyusb/tinyusb-stack/lib/networking",
        "../tinyusb/config/freertos/am64x_am243x/ncm_config",
        "../cdn/core_driver/common/src",
        "../cdn/core_driver/common/include",
        "../cdn/core_driver/device/src",
        "../cdn/core_driver/device/include",
        "../cdn/include",
        "../cdn/soc/am64x_am243x",
		"../../networking/lwip/lwip-stack/src/include",
		"../../networking/lwip/lwip-stack/src/include/ipv4",
		"../../networking/lwip/lwip-stack/src/include/lwip/apps",
		"../../networking/lwip/lwip-stack/src/include/lwip",
		"../../networking/lwip/lwip-config/am243x/usb/",
		"../../networking/lwip/lwip-port/include/",
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
	release:[
	]
};

const buildOptionCombos = [
    { device: device, cpu: "r5f", cgt: "ti-arm-clang"},
    { device: device, cpu: "r5f", cgt: "gcc-armv7"},
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
