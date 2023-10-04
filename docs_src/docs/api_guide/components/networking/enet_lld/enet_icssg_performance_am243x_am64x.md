# Ethernet Performance{#enetlld_icssg_performance}
[TOC]
# Introduction
This section provides the performance numbers of Ethernet drivers using ICSSG peripheral in MCU+ SDK.

# Setup Details
SOC Details           | Values          |
----------------------|-----------------|
Core                  | R5F             |
Core Operating Speed  | 800 MHz         |
Memory Type           | MSRAM           |
Cache status          | Enabled         |

# TCP/IP Performance

## Configuration Details
Configuration          | Value                    |
--------------------------------|--------------------------|
Processing Core                 | Main R5F0 Core 0         |
Core Frequency                  | 800 MHz                  |
Ethernet Interface Type         | RGMII at 1 Gbps           |
Packet buffer memory      | MSRAM (cached)                      |
Hardware checksum offload | Disabled on both Tx andn Rx Side |
Scatter-gather TX         | Yes                      |
Scatter-gather RX         | Yes                      |
CPDMA interrupt pacing    | Yes                      |
RTOS                            | FreeRTOS                 |
RTOS application                | \ref EXAMPLES_ENET_LWIP_ICSSG example   |
TCP/IP stack                    | LwIP version @VAR_LWIP_VERSION               |
Host PC tool version            | iperf v2.0.10            |
Number of Rx packet buffers     | 32                       |
Number of Tx packet buffers     | 16                       |
\n
### TCP Throughput
<table>
    <tr>
        <td style="text-align: left;"><b>Test</b></td>
        <td style="text-align: center;"><b>Bandwidth \n (Mbps)</b></td>
        <td style="text-align: center;"><b>CPU Load \n (%) </b></td>
    </tr>
    <tr>
        <td>TCP RX</td><td>72</td><td>24</td>
    </tr>
    <tr>
        <td>TCP TX</td><td>80</td><td>24</td>
    </tr>
    <tr>
        <td>TCP Bidirectional</td><td>RX=20 \n TX=71</td><td>30</td>
    </tr>
</table>

<b>Host PC commands:</b>
- TCP Rx and Tx (Sequential):\n
    $```iperf -c <evm_ip> -r```\n
- TCP Rx and Tx (Bidirectional):\n
    $```iperf -c <evm_ip> -d```\n

# See Also
\ref NETWORKING
\ref EXAMPLES_ENET_LWIP_ICSSG