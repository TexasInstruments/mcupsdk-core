
# Full Protocol Stacks from TI {#INDUSTRIAL_COMMS_TI_STACK}

[TOC]

\cond SOC_AM263X
\attention Full Protocol Stacks from TI is under development and not available in this release.
\endcond

\cond SOC_AM64X || SOC_AM243X

Industrial protocol stacks and examples included in this SDK are supported directly by TI powered by Kunbus technology.

Following industrial communication protocols are supported:

## EtherCAT SubDevice

EtherCAT (Ethernet for Control Automation Technology) is a real-time industrial Ethernet standard for industrial automation applications, such as input/output (I/O) devices, sensors and programmable logic controllers (PLCs). It was originally developed by Beckhoff Automation GmbH but is now overseen by the EtherCAT Technology Group that was set up to help with proliferation of the EtherCAT standard. EtherCAT technology adds
certain features on Ethernet and enforces certain configurations to make it a very efficient network technology for automation while fully conforming to the Ethernet specifications. The design of EtherCAT enables any standard PC to be used as an EtherCAT MainDevice and communicate with EtherCAT SubDevices, which are specialized devices compliant with the EtherCAT specification. Together, the MainDevice and SubDevice EtherCAT devices can be used in all devices in the factory network – automation controllers, operator interfaces, remote input/output units, sensors, actuators, drives and others.

### Data Sheet

\cond SOC_AM64X

For information about supported features and key performance parameters, see \htmllink{../industrial_protocol_docs/am64x/ethercat_slave/ethercat_datasheet.html, EtherCAT SubDevice Data Sheet}

\endcond

\cond SOC_AM243X

For information about supported features and key performance parameters, see \htmllink{../industrial_protocol_docs/am243x/ethercat_slave/ethercat_datasheet.html, EtherCAT Slave Data Sheet}

\endcond

### Examples

For more details regarding the pre-integrated stack and example applications, see \ref EXAMPLES_INDUSTRIAL_COMMS_ETHERCAT_SLAVE_DEMOS.

## EtherNet/IP Adapter

EtherNet/IP is a member of a family of network protocols that implements the Common Industrial Protocol (CIP) at its upper layers. EtherNet/IP is the name given to CIP when it is implemented over standard Ethernet as defined by IEEE 802.3. EtherNet/IP is managed by the Open DeviceNet Vendor Association, Inc. (ODVA), which also has responsibility for publishing The EtherNet/IP Specification and coordinating conformance testing. Because EtherNet/IP uses standard Ethernet and TCP/IP technologies, compatibility and coexistence with other applications and protocols is assured.

### Data Sheet

\cond SOC_AM64X

For information about supported features and key performance parameters, see \htmllink{../industrial_protocol_docs/am64x/ethernetip_adapter/ethernetip_datasheet.html, EtherNet/IP Adapter Data Sheet}

\endcond

\cond SOC_AM243X

For information about supported features and key performance parameters, see \htmllink{../industrial_protocol_docs/am243x/ethernetip_adapter/ethernetip_datasheet.html, EtherNet/IP Adapter Data Sheet}

\endcond

### Examples

For more details regarding the pre-integrated stack and example applications, see \ref EXAMPLES_INDUSTRIAL_COMMS_ETHERNETIP_ADAPTER_DEMOS.

## IO-Link Controller

IO-Link (International Electrotechnical Commission [IEC] 61131-9) is an open standards protocol that addresses the need for intelligent control of small devices such as sensors and actuators. This standard provides lowspeed point-to-point serial communication between a device and a controller that normally serves as a gateway to a fieldbus and PLC. The intelligent link established enables ease of communication for data exchange, configuration, and diagnostics. The many advantages of an IO-Link system include standardized wiring, increased data availability, remote monitoring and configuration, simple replacement of devices and advanced diagnostics.

### Data Sheet

\cond SOC_AM64X

For information about supported features and key performance parameters, see \htmllink{../industrial_protocol_docs/am64x/iolink_master/datasheet.html, IO-Link Controller Data Sheet}

\endcond

\cond SOC_AM243X

For information about supported features and key performance parameters, see \htmllink{../industrial_protocol_docs/am243x/iolink_master/datasheet.html, IO-Link Controller Data Sheet}

\endcond

### Examples

For more details regarding the pre-integrated stack and example applications, see \ref EXAMPLES_INDUSTRIAL_COMMS_IOLINK_MASTER_DEMO.

## EtherCAT-IOLink Gateway

EtherCAT-IOLink gateway solution which combines EtherCAT industrial Ethernet fieldbus technology and IO-Link intelligent communication technology.

### Data Sheet

\cond SOC_AM64X

For information about supported features and key performance parameters, see \htmllink{../industrial_protocol_docs/am64x/ethercat_iolink_gateway/ethercat_datasheet.html, EtherCAT-IOLink Gateway Data Sheet}

\endcond

\cond SOC_AM243X

For information about supported features and key performance parameters, see \htmllink{../industrial_protocol_docs/am243x/ethercat_iolink_gateway/ethercat_datasheet.html, EtherCAT-IOLink Gateway Data Sheet}

\endcond

### Examples

For more details regarding the pre-integrated stack and example applications, see \ref EXAMPLES_INDUSTRIAL_COMMS_ETHERCAT_IOLINK_GATEWAY_DEMO.

## Profinet Device {#INDUSTRIAL_COMMS_TI_STACK_PROFINET}

PROFINET is a real-time Ethernet standard for the high-speed, deterministic communications required for a wide range of industrial applications including factory automation, process automation and building automation. Originally developed under the leadership of Siemens GmbH as a network extension to the popular PROFIBUS fieldbus technology, PROFINET is now supported by PROFIBUS & PROFINET International. In 2003, PROFINET was integrated into the IEC 61158 and IEC 61784 standards.

\note
 Starting with MCU+ SDK version 08.04.00, the existing PROFINET RT stack and examples will no longer be available in the SDK.

### Profinet Stack Transition {#INDUSTRIAL_COMMS_TI_STACK_PROFINET_STACK_TRANSITION}

TI is transitioning to a new Profinet stack. Profinet RT will be available by end of 2022. Profinet IRT will be available in Q2 of 2023.

This does not impact EtherCAT, EtherNet/IP, or IO-Link stacks that are already certified and currently available in the SDK.

Following are the stack features for the updated Profinet stack.

#### Supported Functionality

Supported Functionality grouped per Conformance Class, as per \htmllink{https://www.profibus.com/download/profinet-io-conformance-classes, PNIO-CCs_7042_V11_Mar11.pdf}

<table>
<tr><th> **Feature** </th>
<th> **Description** </th>
<th> **Relevant Conformance Class** </th>
<th> **Implementation** </th></tr>
<tr><td colspan="3"><p>PROFINET Specification Version</p></td>
<td><p>2.43</p></td></tr>
<tr><td colspan="3"><p>PROFINET GSDML Specification Version</p></td>
<td><p>2.43</p></td></tr>
<tr><td colspan="3"><p>Netload Class</p></td>
<td><p>I, II, III</p></td></tr>
<tr><td colspan="4"><p><strong>Mandatory Features CC-A</strong></p>
</td></tr>
<tr><td><p>Real Time Cyclic - Class 1</p></td>
<td><p>Unsynchronized Real Time Cyclic Protocol</p></td>
<td><p>CC-A,B,C</p></td>
<td><p>Yes</p></td></tr>
<tr><td><p>Real Time Acyclic</p></td>
<td><p>Real Time Acyclic Protocol</p></td>
<td><p>CC-A,B,C</p></td>
<td><p>Yes</p></td></tr>
<tr><td><p>Device diagnostics/Alarms</p></td>
<td><p>Diagnostics &amp; Maintenance Alarms</p></td>
<td><p>CC-A,B,C</p></td>
<td><p>Yes</p></td></tr>
<tr><td><p>Device Identification (I&amp;M0)</p></td>
<td><p>Supported mandatory Identification Records</p></td>
<td><p>CC-A,B,C</p></td>
<td><p>Yes</p></td></tr>
<tr><td><p>Neighboorhood detection</p></td>
<td><p>LLDP protocol</p></td>
<td><p>CC-A,B,C</p></td>
<td><p>Yes</p></td></tr>
<tr><td><p>Port-related network statuses via PROFINET</p></td>
<td><p>PDEV records</p></td>
<td><p>CC-A,B,C</p></td>
<td><p>Yes</p></td></tr>
<tr><td colspan="4"><p><strong>Optional Features CC-A</strong></p>
</td></tr>
<tr><td><p>Extended Identification &amp; Maintenance</p></td>
<td><p>Supported Extended I&amp;M Records</p></td>
<td><p>CC-A,B,C</p></td>
<td><p>I&amp;M1, 2, 3, 4</p></td></tr>
<tr>
    <td>
    <br>
</td>
<td><p>Not Supported Extended I&amp;M Records</p></td>
<td><p>CC-A,B,C</p></td>
<td><p>I&amp;M5</p></td></tr>
<tr><td><p>Shared Input</p></td>
<td><p>Multiple access to inputs by various controllers</p></td>
<td><p>CC-A,B,C</p></td>
<td><p>Yes</p></td></tr>
<tr><td><p>Shared device</p></td>
<td><p>Distribution of device functions to various controllers</p></td>
<td><p>CC-A,B,C</p></td>
<td><p>Yes</p></td></tr>
<tr><td><p>Device-to-Device communication</p></td>
<td><p>Direct communication between IO-Devices</p></td>
<td><p>CC-A,B,C</p></td>
<td><p>No</p></td></tr>
<tr><td colspan="4"><p><strong>Mandatory Features CC-B</strong></p>
</td></tr>
<tr><td><p>Network diagnostics</p></td>
<td><p>SNMP protocol</p></td>
<td><p>CC-B,C</p></td>
<td><p>Yes</p></td></tr>
<tr><td colspan="4"><p><strong>Optional Features CC-B</strong></p>
</td></tr>
<tr><td><p>Name assignment via DCP, PDEV</p></td>
<td><p>Automatic addressing of devices after device replacement</p></td>
<td><p>CC-B,C</p></td>
<td><p>Yes</p></td></tr>
<tr><td><p>Configuration in Run (CiR)</p></td>
<td><p>Configuration changes during operation</p></td>
<td><p>CC-B,C</p></td>
<td><p>Yes</p></td></tr>
<tr><td><p>Time stamping</p></td>
<td><p>Time stamping of I/O data</p></td>
<td><p>CC-B,C</p></td>
<td><p>Yes</p></td></tr>
<tr><td><p>Fibre-optic cable Support</p></td>
<td><p>Fiber-optic cable diagnostics for POF/HCS</p></td>
<td><p>CC-B,C</p></td>
<td><p>No</p></td></tr>
<tr><td><p>Fast Start-Up</p></td>
<td><p>Fast start-up after voltage recovery for switching operations</p></td>
<td><p>CC-B</p></td>
<td><p>Yes</p></td></tr>
<tr><td><p>Media redundancy protocol</p></td>
<td><p>Higher availability through ring redundancy</p></td>
<td><p>CC-B</p></td>
<td><p>Yes</p></td></tr>
<tr><td><p>System redundancy (mandatory for PA only)</p></td>
<td><p>System redundancy with two I/O Controllers</p></td>
<td><p>CC-B</p></td>
<td><p>Yes</p></td></tr>
<tr><td colspan="4"><p><strong>Mandatory Features CC-C</strong></p>
</td></tr>
<tr><td><p>PROFINET with IRT</p></td>
<td><p>Bandwidth reservation with update rates of 250 us* and higher</p></td>
<td><p>CC-C</p></td>
<td><p>Yes</p></td></tr>
<tr><td colspan="4"><p><strong>Optional Features CC-C</strong></p>
</td></tr>
<tr><td><p>PROFINET with IRT</p></td>
<td><p>Isochronous operation</p></td>
<td><p>CC-C</p></td>
<td><p>Yes</p></td></tr>
<tr>
    <td>
    <br>
</td>
<td><p>Update rates less than 250 us</p></td>
<td><p>CC-C</p></td>
<td><p>No</p></td></tr>
<tr><td><p>DFP</p></td>
<td><p>Optimized IRT mode for line topologies</p></td>
<td><p>CC-C</p></td>
<td><p>No</p></td></tr>
<tr><td><p>Two-way transmission, MRPD</p></td>
<td><p>Higher availability through harmonious redundancy switchover</p></td>
<td><p>CC-C</p></td>
<td><p>No</p></td></tr>
</table>

*number of slots, cyclic data and possibly AR may affect the possible cycle time.

#### Key Performance Parameters

<table>
<tr><th> **Feature** </th>
<th> **Description** </th>
<th> **Setting** </th></tr>
<tr><td rowspan="5"><p>Process Data Image</p></td>
    <td><p>Cyclic Input Data</p></td>
    <td><p>1440 Bytes</p></td>
</tr>
<tr><td><p>Cyclic Output Data</p></td>
    <td><p>1440 Bytes</p></td>
</tr>
<tr><td><p>Supported I/O Data at 8ms cycle time</p></td>
    <td><p>1440 Bytes</p></td>
</tr>
<tr><td><p>Supported I/O Data at 4ms cycle time</p></td>
    <td><p>1440 Bytes</p></td>
</tr>
<tr><td><p>Supported I/O Data at 1ms cycle time</p></td>
    <td><p>1440 Bytes</p></td>
</tr>
<tr><td rowspan="5"><p>Application Relations</p></td>
    <td><p>8 Application Relations per PRU ICSS</p></td>
    <td><p>8 I/O Connections, 1 Supervisor, 1 Supervisor-DA</p></td>
</tr>
<tr><td><p>Input Output Communication Relations</p></td>
    <td><p>8</p></td>
</tr>
<tr><td><p>Consumer Protocol Machines (CPM)</p></td>
    <td><p>8</p></td>
</tr>
<tr><td><p>Provider Protocol Machines (PPM)</p></td>
    <td><p>8</p></td>
</tr>
<tr><td><p>Supported Subslots per Application Relation</p></td>
    <td><p>255</p></td>
</tr>
<tr><td><p>Diagnosis Entries</p></td>
    <td><p>Supported Number of Diagnosis Records</p></td>
    <td><p>256</p></td>
</tr>
<tr><td rowspan="2"><p>EtherNet Link</p></td>
    <td><p>Supported Baud rates</p></td>
    <td><p>100Mbit/s</p></td>
</tr>
<tr><td><p>Supported Duplex Modes</p></td>
    <td><p>Full</p></td>
</tr>
</table>

\endcond


\cond SOC_AM64X || SOC_AM243X

\note The work-around for issue "i2329 - MDIO: MDIO interface corruption (CPSW and PRU-ICSS)" (described in <a href="https://www.ti.com/lit/er/sprz457e/sprz457e.pdf">AM64x/AM243x Processor Silicon Revision 1.0, 2.0 (Rev. E)</a>) is available in this release. Please see \ref INDUSTRIAL_COMMS_MDIO_MANUALMODE_FW_USAGE for more details.

\endcond
