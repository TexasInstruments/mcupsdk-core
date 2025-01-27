\cond THREADX
# NetxDuo {#NETWORKING_NETXDUO}

[TOC]

## Introduction

NetX Duo embedded TCP/IP network stack is Eclipse Foundation's advanced, industrial grade dual IPv4 and IPv6 TCP/IP network stack that is designed specifically for deeply embedded, real-time, and IoT applications. NetX Duo provides embedded applications with core network protocols such as IPv4, IPv6, TCP, and UDP as well as a complete suite of additional, higher-level add-on protocols. NetX Duo offers security via additional add-on security products, including NetX Duo Secure IPsec and NetX Duo Secure SSL/TLS/DTLS. All of this combined with a small footprint, fast execution, and superior ease-of-use make NetX Duo the ideal choice for the most demanding embedded IoT applications.

In this SDK, NetxDuo is integrated to work with the Enet driver, which can be configured from within SysConfig. To use NetxDuo in your application, just add the NetxDuo module under `NETWORKING` in the SysConfig panel.

- \ref EXAMPLES_ENET_NETXDUO_CPSW_MAC
- \ref EXAMPLES_ENET_NETXDUO_CPSW_SWITCH
- \ref EXAMPLES_ENET_NETXDUO_ICSSG_MAC
- \ref EXAMPLES_ENET_NETXDUO_ICSSG_SWITCH
- \ref EXAMPLES_ENET_NETXDUO_CPSW_TCPCLIENT
- \ref EXAMPLES_ENET_NETXDUO_CPSW_TCPSERVER
- \ref EXAMPLES_ENET_NETXDUO_CPSW_UDPCLIENT

For the full NetxDuo documentation, please refer to the below table.

## Additional References {#ECLIPSE_THREADX_FILEX_ADDITIONAL_REFERENCES}

<table>
<tr>
    <th>Website Link
    <th>Description
</tr>
<tr>
    <td>[NetxDuo TCP/IP stack](https://github.com/eclipse-threadx/rtos-docs/blob/main/rtos-docs/netx-duo/overview-netx-duo.md)
    <td>Complete documentation of the NetxDuo TCP/IP network stack and API references
</tr>
</table>
\endcond