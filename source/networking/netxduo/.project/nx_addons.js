

files = [

    // auto-ip
    "nx_auto_ip.c",

    // azure-iot
    // "nx_azure_iot.c",
    // "nx_azure_iot_adu_agent.c",
    // "nx_azure_iot_adu_root_key.c",
    // "nx_azure_iot_hub_client.c",
    // "nx_azure_iot_hub_client_properties.c",
    // "nx_azure_iot_json_reader.c", 
    // "nx_azure_iot_json_writer.c",
    // "nx_azure_iot_provisioning_client.c",

    // BSD
    "nxd_bsd.c",

    // Cloud
    "nx_cloud.c",

    // DHCP
    "nxd_dhcp_client.c",
    "nxd_dhcp_server.c",
    "nxd_dhcpv6_client.c",
    "nxd_dhcpv6_server.c",

    // DNS
    "nxd_dns.c",

    // FTP
    "nxd_ftp_client.c",
    "nxd_ftp_server.c",

    // HTTP
    "nxd_http_client.c",
    "nxd_http_server.c",

    // MDNS
    "nxd_mdns.c",

    // MQTT
    "nxd_mqtt_client.c",

    // NAT
    "nx_nat.c",

    // POP3
    "nxd_pop3_client.c",

    // PPP
    "nx_ppp.c",

    // PPPOE
    "nx_pppoe_client.c",
    "nx_pppoe_server.c",

    // PTP
    "nxd_ptp_client.c",

    // RTP
    "nx_rtp_sender.c",

    // RTSP
    "nx_rtsp_server.c",

    // SMTP
    "nxd_smtp_client.c",

    // SNMP
    "nx_des.c",
    "nxd_snmp.c",
    "nx_sha1.c",

    // SNTP
    "nxd_sntp_client.c",

    // TELNET
    "nxd_telnet_client.c",
    "nxd_telnet_server.c",

    // TFTP
    "nxd_tftp_client.c",
    "nxd_tftp_server.c",

    // Web
    "nx_tcpserver.c",
    "nx_web_http_client.c",
    "nx_web_http_server.c",

    // Web socket
    "nx_sha1.c",
    "nx_websocket_client.c"
];


file_dirs = [
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/auto_ip",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/azure_iot",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/BSD",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/cloud",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/dhcp",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/dns",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/ftp",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/http",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/mdns",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/mqtt",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/nat",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/pop3",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/ppp",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/pppoe",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/ptp",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/rtp",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/rtsp",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/smtp",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/snmp",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/sntp",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/telnet",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/tftp",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/web",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/websocket",
];

includes = [
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/auto_ip",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/azure_iot",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/BSD",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/cloud",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/dhcp",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/dns",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/ftp",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/http",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/mdns",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/mqtt",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/nat",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/pop3",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/ppp",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/pppoe",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/ptp",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/rtp",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/rtsp",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/smtp",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/snmp",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/sntp",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/telnet",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/tftp",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/web",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/websocket",
    "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/azure_iot/azure-sdk-for-c/sdk/inc",
];

module.exports = {
    files,
    file_dirs,
    includes
};