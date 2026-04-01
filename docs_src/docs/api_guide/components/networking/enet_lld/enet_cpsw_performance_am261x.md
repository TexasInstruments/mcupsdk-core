# Ethernet Performance on AM261x {#enetlld_performance}
[TOC]
# Introduction
This section provides the performance numbers of Ethernet drivers using CPSW peripheral in MCU+ SDK

# Setup Details
SOC Details           | Values          |
----------------------|-----------------|
Core                  | R5F             |
Core Operating Speed  | 400 MHz         |
Memory Type           | MSRAM             |
Cache status          | Enabled         |

# Layer 2 Performance
  \imageStyle{CPSW_latency_measurement_diagram.bmp,width:30%}
  \image html CPSW_latency_measurement_diagram.bmp Latency defination

## Configuration Details
Configuration          | Value                    |
--------------------------------|--------------------------|
Processing Core                 | Main R5F0 Core 0         |
Core Frequency                  | 400 MHz                  |
Ethernet Interface Type         | RGMII at 1 Gbps           |
Packet buffer memory      | MSRAM (un-cached)                      |
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
        <td style="text-align: center;"><b>PHY (DP83869HM)Latency\n (from datasheet) in ns</b></td>
        <td style="text-align: center;"><b>Total Latency \n (ns)</b></td>
    </tr>
    <tr>
        <td>RX Latency</td>
        <td>4756</td>
        <td>193</td>
        <td>4949</td>
    </tr>
    <tr>
        <td>TX Latency</td>
        <td>13225</td>
        <td>384</td>
        <td>13609</td>
    </tr>
</table>

### Layer-2 Hardware Switching Latency for CPSW

Configuration                   | Value                    |
--------------------------------|--------------------------|
EVM Type                        | AM261x-SOM               |
CPSW CPPI Frequency             | 320 MHz                  |
Ethernet Interface Type         | RGMII                    |
PHY model number                | DP83869                  |
Ethernet Cable Spec             | CAT-6                    |
Sample size                     | 10 Million Packets       |
RTOS application                | Modified \ref EXAMPLES_ENET_LAYER2_CPSW_SWITCH \n example   |
\n

### Test Setup

\imageStyle{CPSW_Latency_measurement.png,width:15==20%}
\image html CPSW_Latency_measurement.png "Connections for Latency Measurement"

### CPSW Latency for RGMII
<table>
    <tr>
        <td rowspan="2" style="text-align: left;"><b>Datagram Length</b></td>
        <td colspan="2" style="text-align: center;"><b>Latency @ 1Gbps link (in us)</b></td>
        <td colspan="2" style="text-align: center;"><b>Latency @ 100Mbps link (in us)</b></td>
        <td colspan="2" style="text-align: center;"><b>Latency @ 10Mbps link (in us)</b></td>
    </tr>
    <tr>
        <td><b>Without Cut-Through</b></td><td><b>With Cut-Through</b></td>
        <td><b>Without Cut-Through</b></td><td><b>With Cut-Through</b></td>
        <td><b>Without Cut-Through</b></td><td><b>With Cut-Through</b></td>
    </tr>
    <tr>
        <td><b>64 Bytes</b></td>
        <td>1.8</td><td>1.7</td>
        <td>7.9</td><td>7.8</td>
        <td>73.4</td><td>71.9</td>
    </tr>
    <tr>
        <td><b>256 Bytes</b></td>
        <td>3.1</td><td>2.0</td>
        <td>23.0</td><td>13.8</td>
        <td>227.3</td><td>135.6</td>
    </tr>
    <tr>
        <td><b>512 Bytes</b></td>
        <td>5.2</td><td>2.0</td>
        <td>43.5</td><td>13.8</td>
        <td>432.0</td><td>135.6</td>
    </tr>
    <tr>
        <td><b>1518 Bytes</b></td>
        <td>13.2</td><td>2.0</td>
        <td>124.0</td><td>13.8</td>
        <td>1237.0</td><td>135.6</td>
    </tr>
</table>

# TCP/IP Performance

## Configuration Details
Configuration          | Value                    |
--------------------------------|--------------------------|
Processing Core                 | Main R5F0 Core 0         |
Core Frequency                  | 400 MHz                  |
Ethernet Interface Type         | RGMII at 1 Gbps           |
Packet buffer memory      | MSRAM (cached)                      |
Hardware checksum offload | Enabled on Tx side \n Enabled on Rx Side |
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
        <td>TCP RX</td><td>195</td><td>98</td>
    </tr>
    <tr>
        <td>TCP TX</td><td>158</td><td>100</td>
    </tr>
    <tr>
        <td>TCP Bidirectional</td><td>RX=73 \n TX=87</td><td>100</td>
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
        <td colspan="3" style="text-align: center;"><b>Datagram Length = 64B </b></td>
        <td colspan="3" style="text-align: center;"><b>Datagram Length = 256B</b></td>
        <td colspan="3" style="text-align: center;"><b>Datagram Length = 512B</b></td>
        <td colspan="3" style="text-align: center;"><b>Datagram Length = 1470B</b></td>
    </tr>
    <tr>
        <td><b>Bandwidth (Mbps)</b></td><td><b>CPU Load (%)</b></td><td><b>Packet Loss (%)</b></td>
        <td><b>Bandwidth (Mbps)</b></td><td><b>CPU Load (%)</b></td><td><b>Packet Loss (%)</b></td>
        <td><b>Bandwidth (Mbps)</b></td><td><b>CPU Load (%)</b></td><td><b>Packet Loss (%)</b></td>
        <td><b>Bandwidth (Mbps)</b></td><td><b>CPU Load (%)</b></td><td><b>Packet Loss (%)</b></td>
    </tr>
    <tr>
        <td rowspan="3">UDP RX</td>
        <td>5</td><td>41</td><td>0.0</td>
        <td>5</td><td>20</td><td>0.0</td>
        <td>25</td><td>48</td><td>0.0</td>
        <td>50</td><td>42</td><td>0.0</td>
    </tr>
    <tr>
        <td>10</td><td>72</td><td>0.0</td>
        <td>15</td><td>53</td><td>0.0</td>
        <td>50</td><td>85</td><td>0.7</td>
        <td>60</td><td>48</td><td>0.0</td>
    </tr>
    <tr>
        <td>15</td><td>91</td><td>7.3</td>
        <td>25</td><td>81</td><td>0.15</td>
        <td>55</td><td>95</td><td>0.4</td>
        <td>95</td><td>72</td><td>0.0</td>
    </tr>
    <tr>
        <td>UDP RX (Max)</td>
        <td>13</td><td>91</td><td>1.3</td>
        <td>30</td><td>96</td><td>3.4</td>
        <td>58</td><td>97</td><td>1.1</td>
        <td>130.8</td><td>97</td><td>1.4</td>
    </tr>
    <tr>
        <td>UDP TX (Max)</td>
        <td>23.7</td><td>100</td><td>0.0</td>
        <td>54.1</td><td>100</td><td>0.0</td>
        <td>108</td><td>100</td><td>0.0</td>
        <td>311</td><td>100</td><td>0.0</td>
    </tr>
</table>

<b>Host PC commands:</b>
- Test with datagram length of 64B:\n
     $```iperf -c <evm_ip> -u -l64 -b <bw> -r```\n
where \<bw\> is 5M, 10M, 15M, etc\n

- Test with datagram length of 256B:\n
     $```iperf -c <evm_ip> -u -l256 -b <bw> -r```\n
where \<bw\> is 25M, 50M, 100M, etc\n

- Test with datagram length of 512B:\n
     $```iperf -c <evm_ip> -u -l512 -b <bw> -r```\n
where \<bw\> is 25M, 50M, 100M, etc\n

- Test with datagram length of 1470B (max):\n
     $```iperf -c <evm_ip> -u -b <bw> -r```\n
where \<bw\> is 25M, 50M, 100M, etc\n

# XIP Performance{#networking_xip_performance}

Lwip Application size: 793KB

FLASH: 198KB
<table>
    <tr>
        <td style="text-align: center;"><b>Region</b></td>
        <td style="text-align: center;"><b>Memory</b></td>
    </tr>
    <tr>
        <td>.text</td>
        <td>156KB</td>
    </tr>
    <tr>
        <td>.rodata </td>
        <td>42KB</td>
    </tr>
</table>

OCRAM: 595KB
<table>
    <tr>
        <td style="text-align: center;"><b>Region</b></td>
        <td style="text-align: center;"><b>Memory</b></td>
    </tr>
    <tr>
        <td>.bss</td>
        <td>471KB</td>
    </tr>
    <tr>
        <td>.sysmem</td>
        <td>34KB</td>
    </tr>
    <tr>
        <td>.data</td>
        <td>23KB</td>
    </tr>
    <tr>
        <td>.stack</td>
        <td>8KB</td>
    </tr>
    <tr>
        <td>misc</td>
        <td>59KB</td>
    </tr>
</table>


<table>
    <tr>
        <td style="text-align: center;"><b>Board POV</b></td>
        <td style="text-align: center;"><b>MSRAM - OOB</b></td>
        <td style="text-align: center;"><b>XIP : text and ro data in flash</b></td>
        <td style="text-align: center;"><b>Opti-flash: RL2 8KB</b></td>
        <td style="text-align: center;"><b>Opti-flash: RL2 16KB</b></td>
        <td style="text-align: center;"><b>Opti-flash: RL2 32KB</b></td>
        <td style="text-align: center;"><b>Opti-flash: RL2 64KB</b></td>
        <td style="text-align: center;"><b>Opti-flash: RL2 128KB</b></td>
    </tr>
    <tr>
        <td>UDP Tx </td>
        <td>273Mbps at 100% CPU load</td>
        <td>145Mbps at 100% CPU load</td>
        <td>224Mbps at 100% CPU load</td>
        <td>252Mbps at 100% CPU load</td>
        <td>270Mbps at 100% CPU load</td>
        <td>273Mbps at 100% CPU load</td>
        <td>273Mbps at 100% CPU load</td>
    </tr>
    <tr>
        <td>UDP Rx</td>
        <td>116Mbps ~1% pkt drop 86% CPU load</td>
        <td>45Mbps  ~1% pkt drop 98% CPU load</td>
        <td>64Mbps  ~1% pkt drop 99% CPU load</td>
        <td>92Mbps  ~1% pkt drop 97% CPU load</td>
        <td>108Mbps ~1% pkt drop 89% CPU load</td>
        <td>110Mbps ~1% pkt drop 86% CPU load</td>
        <td>115Mbps ~1% pkt drop 89% CPU load</td>
    </tr>
</table>

# See Also
\ref NETWORKING
\ref EXAMPLES_ENET_LWIP_CPSW
