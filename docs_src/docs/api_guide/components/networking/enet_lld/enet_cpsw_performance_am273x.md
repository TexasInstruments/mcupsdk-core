# Ethernet Performance on AM273x {#enetlld_performance}
[TOC]
# Introduction
This section provides the performance numbers of Ethernet drivers using CPSW peripheral in MCU+ SDK

# Setup Details
SOC Details           | Values          |
----------------------|-----------------|
Core                  | R5F             |
Core Operating Speed  | 400 MHz         |
Memory Type           | MSS L2 RAM             |
Cache status          | Enabled         |

# Layer 2 Performance
  \imageStyle{CPSW_latency_measurement_diagram.bmp,width:30%}
  \image html CPSW_latency_measurement_diagram.bmp Latency defination

## Configuration Details
Configuration          | Value                    |
--------------------------------|--------------------------|
Processing Core                 | Main R5F0 Core 0         |
Core Frequency                  | 400 MHz                  |
Ethernet Interface Type         | RGMII at 100 mbps           |
Packet buffer memory      | MSS L2 RAM (un-cached)         |
Scatter-gather TX         | Yes                      |
Scatter-gather RX         | Yes                      |
CPDMA interrupt pacing    | Yes                      |
RTOS                            | FreeRTOS                 |
RTOS application                | Modified \ref EXAMPLES_ENET_CPSW_LOOPBACK \n example   |
Host PC tool version            | nload                   |
Rx packet length     | 200 B                       |
Tx packet length     | 200 B                       |
\n

## Layer 2 Latency
<table>
    <tr>
        <td style="text-align: center;"><b>Parameter</b></td>
        <td style="text-align: center;"><b>CPU<->CPSW \n Latency Value (ns)</b></td>
        <td style="text-align: center;"><b>PHY latency\n (from datasheet) in ns</b></td>
        <td style="text-align: center;"><b>Total Latency \n (ns)</b></td>
    </tr>
    <tr>
        <td>RX Latency</td>
        <td>4000</td>
        <td>193</td>
        <td>4193</td>
    </tr>
    <tr>
        <td>TX Latency</td>
        <td>14000</td>
        <td>384</td>
        <td>14384</td>
    </tr>
</table>

## Layer 2 Throughput
<table>
    <tr>
        <td style="text-align: left;"><b>Test</b></td>
        <td style="text-align: center;"><b>Bandwidth \n (Mbps)</b></td>
        <td style="text-align: center;"><b>CPU Load \n (%) </b></td>
        <td style="text-align: center;"><b>Packet Size \n (bytes)</b></td>
    </tr>
    <tr>
        <td>Layer 2 Transmission</td><td>950</td><td>24</td><td>1500</td>
    </tr>
</table>


# TCP/IP Performance

## Configuration Details
Configuration          | Value                    |
--------------------------------|--------------------------|
Processing Core                 | Main R5F0 Core 0         |
Core Frequency                  | 400 MHz                  |
Ethernet Interface Type         | RGMII at 100 Mbps           |
Packet buffer memory      | MSS L2 RAM (cached)                      |
Hardware checksum offload | Disabled on Tx side \n Disabled on Rx Side |
Scatter-gather TX         | Yes                      |
Scatter-gather RX         | Yes                      |
CPDMA interrupt pacing    | Yes                      |
RTOS                            | FreeRTOS                 |
RTOS application                | \ref EXAMPLES_ENET_LWIP_CPSW example   |
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
        <td>TCP RX</td><td>95</td><td>74.7</td>
    </tr>
    <tr>
        <td>TCP TX</td><td>95</td><td>79.5</td>
    </tr>
    <tr>
        <td>TCP Bidirectional</td><td>RX=63 \n TX=63</td><td>100</td>
    </tr>
</table>

<b>Host PC commands:</b>
- TCP Rx and Tx (Sequential):\n
    $```iperf -c <evm_ip> -r```\n
- TCP Rx and Tx (Bidirectional):\n
    $```iperf -c <evm_ip> -d```\n

### UDP Throughput
<table>
    <tr>
        <td rowspan="2" style="text-align: left;"><b>Test</b></td>
        <td colspan="3" style="text-align: center;"><b>Datagram Length = 256B</b></td>
        <td colspan="3" style="text-align: center;"><b>Datagram Length = 512B</b></td>
        <td colspan="3" style="text-align: center;"><b>Datagram Length = 1470B</b></td>
    </tr>
    <tr>
        <td><b>Bandwidth (Mbps)</b></td><td><b>CPU Load (%)</b></td><td><b>Packet Loss (%)</b></td>
        <td><b>Bandwidth (Mbps)</b></td><td><b>CPU Load (%)</b></td><td><b>Packet Loss (%)</b></td>
        <td><b>Bandwidth (Mbps)</b></td><td><b>CPU Load (%)</b></td><td><b>Packet Loss (%)</b></td>
    </tr>
    <tr>
        <td rowspan="3">UDP RX</td>
        <td>25</td><td>99</td><td>0.0</td>
        <td>25</td><td>56</td><td>0.0</td>
        <td>25</td><td>25</td><td>0.0</td>
    </tr>
    <tr>
        <td>50</td><td>100</td><td>64</td>
        <td>50</td><td>100</td><td>3.4</td>
        <td>50</td><td>46</td><td>0.0</td>
    </tr>
    <tr>
        <td>96</td><td>100</td><td>80</td>
        <td>96</td><td>100</td><td>100</td>
        <td>96</td><td>81</td><td>0.0</td>
    </tr>
    <tr>
        <td>UDP RX (Max)</td>
        <td>Not Tested</td><td>-</td><td>-</td>
        <td>Not Tested</td><td>-</td><td>-</td>
        <td>96</td><td>81</td><td>0.0</td>
    </tr>
    <tr>
        <td>UDP TX (Max)</td>
        <td>Not Tested</td><td>-</td><td>-</td>
        <td>Not Tested</td><td>-</td><td>-</td>
        <td>95.7</td><td>55</td><td>0.0</td>
    </tr>
</table>

<b>Host PC commands:</b>
- Test with datagram length of 64B:\n
     $```iperf -c <evm_ip> -u -l64 -b<bw> -r```\n
where \<bw\> is 25M, 50M, 100M, etc\n

- Test with datagram length of 256B:\n
     $```iperf -c <evm_ip> -u -l256 -b<bw> -r```\n
where \<bw\> is 25M, 50M, 100M, etc\n

- Test with datagram length of 512B:\n
     $```iperf -c <evm_ip> -u -l512 -b<bw> -r```\n
where \<bw\> is 25M, 50M, 100M, etc\n

- Test with datagram length of 1470B (max):\n
     $```iperf -c <evm_ip> -u -b<bw> -r```\n
where \<bw\> is 25M, 50M, 100M, etc\n

# See Also
\ref NETWORKING
\ref EXAMPLES_ENET_LWIP_CPSW