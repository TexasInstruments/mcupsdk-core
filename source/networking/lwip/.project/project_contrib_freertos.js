let path = require('path');
const _ = require('lodash');

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
    ],
};

const deviceSpecificIncludes = {
    am243x : [
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-config/am243x",
    ],
    am64x : [
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-config/am64x",
    ],
    am263x : [
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-config/am263x",
    ],
    am273x : [
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-config/am273x",
    ],
    awr294x : [
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-config/awr294x",
    ],
};


const cflags = {
    common: [
        "-Wno-extra",
        "-Wvisibility",
    ],
    release: [
        "-Oz",
        "-flto",
    ],
};

const deviceSpecific_cflags = {
     am243x : [
        "-mthumb",
        "-fno-strict-aliasing",
    ],
    am64x : [
        "-mthumb",
        "-fno-strict-aliasing",
    ],
    am263x : [
    ],
    am273x : [
        "-fno-strict-aliasing",
    ],
    awr294x : [
        "-fno-strict-aliasing",
    ],
};


const buildOptionCombos = [
    { device: "am263x", cpu: "r5f", cgt: "ti-arm-clang"},
    { device: "am243x", cpu: "r5f", cgt: "ti-arm-clang"},
    { device: "am273x", cpu: "r5f", cgt: "ti-arm-clang"},
    { device: "am64x",  cpu: "r5f", cgt: "ti-arm-clang"},
    { device: "awr294x", cpu: "r5f", cgt: "ti-arm-clang"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "library";
    property.name = "lwip-contrib-freertos";
    property.tag = "contrib-freertos";
    property.isInternal = false;
    deviceBuildCombos = []
    for (buildCombo of buildOptionCombos)
    {
        if (buildCombo.device === device)
        {
            deviceBuildCombos.push(buildCombo)
        }
    }
    property.buildOptionCombos = deviceBuildCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;

    includes.common = _.union(includes.common, deviceSpecificIncludes[device]);
    build_property.includes = includes;

    cflags.common = _.union(cflags.common, deviceSpecific_cflags[device]);
    build_property.cflags = cflags;

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
