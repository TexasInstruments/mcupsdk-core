let path = require('path');
const _ = require('lodash');

const files = {
    common: [
        /* lwipcore_SRCS */
        "init.c"           ,
        "def.c"            ,
        "dns.c"            ,
        "inet_chksum.c"    ,
        "ip.c"             ,
        "mem.c"            ,
        "memp.c"           ,
        "netif.c"          ,
        "pbuf.c"           ,
        "raw.c"            ,
        "stats.c"          ,
        "sys.c"            ,
        "altcp.c"          ,
        "altcp_alloc.c"    ,
        "altcp_tcp.c"      ,
        "tcp.c"            ,
        "tcp_in.c"         ,
        "tcp_out.c"        ,
        "timeouts.c"       ,
        "udp.c"            ,

        /* lwipcore4_SRCS */
        "acd.c"            ,
        "autoip.c"         ,
        "dhcp.c"           ,
        "etharp.c"         ,
        "icmp.c"           ,
        "igmp.c"           ,
        "ip4_frag.c"       ,
        "ip4.c"            ,
        "ip4_addr.c"       ,

        /* lwipcore6_SRCS */
        "dhcp6.c"          ,
        "ethip6.c"         ,
        "icmp6.c"          ,
        "inet6.c"          ,
        "ip6.c"            ,
        "ip6_addr.c"       ,
        "ip6_frag.c"       ,
        "mld6.c"           ,
        "nd6.c"            ,

        /* lwipapi_SRCS */
        "api_lib.c"        ,
        "api_msg.c"        ,
        "err.c"            ,
        "if_api.c"         ,
        "netbuf.c"         ,
        "netdb.c"          ,
        "netifapi.c"       ,
        "sockets.c"        ,
        "tcpip.c"          ,

        /* lwipnetif_SRCS */
        "ethernet.c"       ,
        "bridgeif.c"       ,
        "bridgeif_fdb.c"   ,
        "slipif.c"         ,

        /* lwipsixlowpan_SRCS */
        "lowpan6_common.c" ,
        "lowpan6.c"        ,
        "lowpan6_ble.c"    ,
        "zepif.c"          ,

        /* lwipppp_SRCS */
        "auth.c"           ,
        "ccp.c"            ,
        "chap-md5.c"       ,
        "chap_ms.c"        ,
        "chap-new.c"       ,
        "demand.c"         ,
        "eap.c"            ,
        "ecp.c"            ,
        "eui64.c"          ,
        "fsm.c"            ,
        "ipcp.c"           ,
        "ipv6cp.c"         ,
        "lcp.c"            ,
        "magic.c"          ,
        "mppe.c"           ,
        "multilink.c"      ,
        "ppp.c"            ,
        "pppapi.c"         ,
        "pppcrypt.c"       ,
        "pppoe.c"          ,
        "pppol2tp.c"       ,
        "pppos.c"          ,
        "upap.c"           ,
        "utils.c"          ,
        "vj.c"             ,
        "arc4.c"           ,
        "des.c"            ,
        "md4.c"            ,
        "md5.c"            ,
        "sha1.c"           ,

        /* lwipsnmp_SRCS */
        "snmp_asn1.c"                ,
        "snmp_core.c"                ,
        "snmp_mib2.c"                ,
        "snmp_mib2_icmp.c"           ,
        "snmp_mib2_interfaces.c"     ,
        "snmp_mib2_ip.c"             ,
        "snmp_mib2_snmp.c"           ,
        "snmp_mib2_system.c"         ,
        "snmp_mib2_tcp.c"            ,
        "snmp_mib2_udp.c"            ,
        "snmp_snmpv2_framework.c"    ,
        "snmp_snmpv2_usm.c"          ,
        "snmp_msg.c"                 ,
        "snmpv3.c"                   ,
        "snmp_netconn.c"             ,
        "snmp_pbuf_stream.c"         ,
        "snmp_raw.c"                 ,
        "snmp_scalar.c"              ,
        "snmp_table.c"               ,
        "snmp_threadsync.c"          ,
        "snmp_traps.c"               ,

        /* lwiphttp_SRCS */
        "altcp_proxyconnect.c"       ,
        "fs.c"                       ,
        "http_client.c"              ,
        "httpd.c"                    ,

        /* lwipmakefsdata_SRCS */
        /* "makefsdata.c"      */    ,

        /* lwipiperf_SRCS */
        "lwiperf.c"                  ,

        /* lwipsmtp_SRCS */
        "smtp.c"                     ,

        /* lwipsntp_SRCS */
        "sntp.c"                     ,

        /* lwipmdns_SRCS */
        "mdns.c"                     ,

        /* lwipnetbios_SRCS */
        "netbiosns.c"                ,

        /* lwiptftp_SRCS */
        "tftp.c"              ,

        /* lwipmqtt_SRCS */
        "mqtt.c"                     ,

        /* lwipmbedtls_SRCS */
        "altcp_tls_mbedtls_mem.c"    ,
        "snmpv3_mbedtls.c"           ,

        "sys_arch.c",

    ],
};
const filedirs = {
    common: [
       // picked from lwip apps
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/src/apps/smtp",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/src/apps/tftp",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/src/apps/mqtt",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/src/apps/tftp",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/src/apps/netbiosns",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/src/apps/mdns",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/src/apps/altcp_tls",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/src/apps/http",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/src/apps/http/makefsdata",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/src/apps/lwiperf",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/src/apps/snmp",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/src/apps/sntp",

       // picked from lwip netif
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/src/netif",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/src/netif/ppp",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/src/netif/ppp/polarssl",

       // picked from lwip core
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/src/core",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/src/core/ipv6",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/src/core/ipv4",

       // picked from lwip api
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/src/api",

       // FreRTOS lwip port
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-port/nortos/src",
       "${MCU_PLUS_SDK_PATH}/source/networking/mbedtls_library/mbedtls/library",
       "${MCU_PLUS_SDK_PATH}/source/networking/mbedtls_library/mbedtls/include",

    ],
};

const includes = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/src/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-port/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-port/nortos/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/mbedtls_library/mbedtls/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/mbedtls_library/mbedtls/library",
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
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-config/am263x/enet",
    ],
    am263px : [
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-config/am263px/enet",
    ],
    am261x : [
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-config/am261x/enet",
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
    am263px : [
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
    { device: "am263px", cpu: "r5f", cgt: "ti-arm-clang"},
    { device: "am261x", cpu: "r5f", cgt: "ti-arm-clang"},
    { device: "am243x", cpu: "r5f", cgt: "ti-arm-clang"},
    { device: "am243x", cpu: "r5f", cgt: "gcc-armv7"},
    { device: "am273x", cpu: "r5f", cgt: "ti-arm-clang"},
    { device: "am64x",  cpu: "r5f", cgt: "ti-arm-clang"},
    { device: "awr294x", cpu: "r5f", cgt: "ti-arm-clang"},
];

function getComponentProperty(device) {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "library";
    property.name = "lwip-nortos";
    property.tag  = "stack_nortos";
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