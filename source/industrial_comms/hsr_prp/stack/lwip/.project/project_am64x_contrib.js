let path = require('path');

let device = "am64x";

const files = {
    common: [
        "fs_example.c",
        "ssi_example.c",
        "lwiperf_example.c",
      /*  "mdns_example.c", */
        "mqtt_example.c",
        "pppos_example.c",
        "lwip_prvmib.c",
        "snmpv3_dummy.c",
        "snmp_example.c",
      /* "tftp_example.c", */

        "httpserver-netconn.c",
        "chargen.c",
        "udpecho.c",
        "tcpecho.c",
        "shell.c",
        "udpecho_raw.c",
        "tcpecho_raw.c",
        "netio.c",
        "ping.c",
        "socket_examples.c",
        "rtp.c",
   ],
};

const filedirs = {
    common: [
       // picked from lwip apps
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-contrib/examples",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-contrib/apps",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-contrib/addons",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-contrib/examples/httpd/fs_example",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-contrib/examples/httpd/ssi_example",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-contrib/examples/lwiperf",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-contrib/examples/mdns",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-contrib/examples/mqtt",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-contrib/examples/ppp",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-contrib/examples/snmp/snmp_private_mib",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-contrib/examples/snmp/snmp_v3",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-contrib/examples/snmp",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-contrib/examples/sntp",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-contrib/examples/tftp",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-contrib/apps/httpserver",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-contrib/apps/chargen",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-contrib/apps/udpecho",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-contrib/apps/tcpecho",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-contrib/apps/shell",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-contrib/apps/udpecho_raw",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-contrib/apps/tcpecho_raw",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-contrib/apps/netio",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-contrib/apps/ping",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-contrib/apps/socket_examples",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-contrib/apps/rtp",
    ],
};

const includes = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/src/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-port/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-port/freertos/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/lwipif/inc",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-contrib",
        "${MCU_PLUS_SDK_PATH}/source/industrial_comms/hsr_prp/stack/lwip/lwip-config/am64x",
    ],
};


const cflags = {
    common: [
        "-Wno-extra",
        "-Wvisibility",
        "-mthumb",
        "-fno-strict-aliasing",
    ],
    release: [
        "-Oz",
    ],
};

const buildOptionCombos = [
    { device: device, cpu: "r5f", cgt: "ti-arm-clang", os: "freertos"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "library";
    property.name = "hsr_prp_lwip_contrib";
    property.tag = "contrib";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.includes = includes;
    build_property.cflags = cflags;

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
