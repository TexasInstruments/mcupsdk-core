# Lightweight IP (LwIP) {#NETWORKING_LWIP}

[TOC]

## Introduction
LwIP is the TCP/IP networking protocol  stack supported and integrated into SDK in its 'venilla' version.

As per [LwIP project page] (http://savannah.nongnu.org/projects/lwip/)

> The focus of the lwIP TCP/IP implementation is to reduce resource usage while still having a full scale TCP. This makes lwIP suitable for use in embedded systems with tens of kilobytes of free RAM and room for around 40 kilobytes of code ROM.
>
> Main features include:
> - Protocols: IP, IPv6, ICMP, ND, MLD, UDP, TCP, IGMP, ARP, PPPoS, PPPoE
> - DHCP client, DNS client (incl. mDNS hostname resolver), AutoIP/APIPA (Zeroconf), SNMP agent (v1, v2c, v3, private MIB support & MIB compiler)
> - APIs: specialized APIs for enhanced performance, optional Berkeley-alike socket API
> - Extended features: IP forwarding over multiple network interfaces, TCP congestion control, RTT estimation and fast recovery/fast retransmit
> - Add-on applications: HTTP(S) server, SNTP client, SMTP(S) client, ping, NetBIOS nameserver, mDNS responder, MQTT client, TFTP server.

Current version that is supported is @VAR_LWIP_VERSION.

## Upgrade
### Upgrade to version 2.2.1
In MCU+ SDK release version 11.02.00, LwIP stack has been upgraded to support LwIP version 2.2.1 (tag: `STABLE-2_2_1_RELEASE`).
Below is the [official release news](https://savannah.nongnu.org/news/?group=lwip) from LwIP-
> lwIP 2.2.1 is now available from the lwIP download area
> or via git (using the STABLE-2_2_1_RELEASE tag) or via the gitweb link.<br>
> This is mostly a bugfix release, summing up all fixes of the last 1.5 years.

\cond SOC_AM273X || SOC_AWR294X || SOC_AM263X || SOC_AM263PX || SOC_AM64X || SOC_AM243X || SOC_AM62DX
Following are the changes done in MCU+ SDK during upgrade -
1. The `mqtt.c` file in `${SDK_INSTALL_PATH}/examples/lwip/cpsw_lwip_mqtt` has been updated to accommodate changes to the `mqtt_connect_client_info_t` struct introduced in the new stack version.
\endcond

This is a smaller bugfix-only release. Contrib has not been changed since 2.1.0.
For a list of the bugs fixed, see `${SDK_INSTALL_PATH}/source/networking/lwip/lwip-stack/CHANGELOG`.

### Upgrade to version 2.2.0
In MCU+ SDK release version 09.01.00, LwIP stack is upgraded to support LwIP version 2.2.0 LwIP (tag:  `STABLE-2_2_0_RELEASE`). Below is the official release news from LwIP-
> lwIP 2.2.0 is now available from the lwIP download area
> or via git (using the STABLE-2_2_0_RELEASE tag) or via this gitweb link:
> There have been some bugs fixed, and some new features were added (most notably full ACD support).
> Additionally, the contents of the contrib repository has been moved to the main repository ('contrib' directory on top level) to make things more consistent, especially with git. The old contrib repository should not be used any more.
> The 2.1.x branch will not be continued, so eventually, all users of 2.1.x might want to upgrade to get bugfixes.

Following are the changes done in MCU+ SDK during upgrade -
1. Deleted `lwip-contrib` directory in `${SDK_INSTALL_PATH}/source/networking/lwip`.
2. In  makefiles, all the references to `lwip-contrib` directory is replaced with new `contrib` directory present in  `${SDK_INSTALL_PATH}/source/networking/lwip-stack/contrib`.
3. Added  `acd.c` (newly added file in stack) of lwip stack library makefile
4. All the references to file `tftp_server.c` is replaced with `tftp.c`. `tftp_server.c`  file is renamed in latest lwip stack,  and  has functionality of both tftp server and client.
5. No change to pre-compiled `lwip-contrib` library status. It is pre-compiled now using new location for `contrib` directory and continue to exists.

If you have a custom makefile used to compile lwip stack, please ensure to make the above changes to use new version of LwIP stack.

\subpage NETWORKING_LWIP_STATIC_IP.