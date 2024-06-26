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
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/contrib/examples",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/contrib/apps",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/contrib/addons",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/contrib/examples/httpd/fs_example",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/contrib/examples/httpd/ssi_example",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/contrib/examples/lwiperf",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/contrib/examples/mdns",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/contrib/examples/mqtt",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/contrib/examples/ppp",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/contrib/examples/snmp/snmp_private_mib",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/contrib/examples/snmp/snmp_v3",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/contrib/examples/snmp",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/contrib/examples/sntp",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/contrib/examples/tftp",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/contrib/apps/httpserver",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/contrib/apps/chargen",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/contrib/apps/udpecho",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/contrib/apps/tcpecho",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/contrib/apps/shell",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/contrib/apps/udpecho_raw",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/contrib/apps/tcpecho_raw",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/contrib/apps/netio",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/contrib/apps/ping",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/contrib/apps/socket_examples",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/contrib/apps/rtp",
    ],
};

const includes = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/src/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-port/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-port/freertos/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/lwipif/inc",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/contrib",
    ],
};

const deviceSpecificIncludes = {
    am263x : [
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-config/am263x/icss_emac",
    ],
    am263px : [
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-config/am263px/icss_emac",
    ],
    am261x : [
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-config/am261x/icss_emac",
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
    am263x : [
    ],
    am263px : [
    ],
    am261x : [
    ],
};


const buildOptionCombos = [
    { device: "am263x", cpu: "r5f", cgt: "ti-arm-clang"},
    { device: "am263px", cpu: "r5f", cgt: "ti-arm-clang"},
    { device: "am261x", cpu: "r5f", cgt: "ti-arm-clang"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "library";
    property.name = "lwip-contrib-freertos-icss_emac";
    property.tag = "contrib-freertos-icss_emac";
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

    if(buildOption.cgt.match(/gcc-armv7*/)){
        cflags.common = [];
        cflags.release = [];
    }
    cflags.common = _.union(cflags.common, deviceSpecific_cflags[device]);
    build_property.cflags = cflags;

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};