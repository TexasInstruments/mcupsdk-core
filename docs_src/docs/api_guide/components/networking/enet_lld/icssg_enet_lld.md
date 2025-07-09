PRU-ICSSG for Ethernet Networking {#ICSSG_ETH_TOP}
======================

# Mode of Operation

ICSSG can be configured to operate in two modes, Dual-MAC and Switch.

It is important to differentiate between these two modes. Dual-MAC mode implements two independent Ethernet MACs using ICSSG Slice 0/1, they have two different MAC, IP addresses and two different instances of the TCP/IP stack, while a Switch presents a single IP and MAC address for any external entity. 

# ICSSG Switch and Dual-MAC architecture

The ICSSG architecture consists of two logical slices to implement the switch functionalities. PRU0 (RX PRU0), TX_PRU0 and RTU_PRU0 is considered as Slice0 and the other cores PRU1 (RX PRU1), TX_PRU1 and RTU_PRU1 form the Slice1.<br>
In Switch mode, each slice handles cross-port transmission: receiving on one port and transmitting on the opposite port. In contrast, in Dual-MAC mode, each slice operates independently, handling both receiving and transmitting on the same port.<br>
This has been achieved by accordingly configuring the TX_MUX_SELn field of MII_RT_TXCFGn memory-mapped register (n= 0,1). For more details, refer TRM section 6.4.14.11 PRU_MII_RT_MII_RT Registers.

<table>
<tr>
    <th>Bit-field
    <th>Switch
    <th>Dual-MAC
</tr>
<tr>
    <td>TX_MUX_SEL0
    <td>1h
    <td>0h
</tr>
<tr>
    <td>TX_MUX_SEL1
    <td>0h
    <td>1h
</tr>
</table>

\imageStyle{icssg_switch_arch.png,width:75%}
\image html icssg_switch_arch.png  **Figure1**: ICSSG Architecture in Switch Mode

 \imageStyle{icssg_dual_mac_arch.png,width:75%}
 \image html icssg_dual_mac_arch.png  **Figure2**: ICSSG Architecture in Dual-MAC Mode

# ICSSG Packet flow paths

There are 3 basic packet flow paths:

- **Host Egress**: Packets are received from Port1/2 by RX_PRU and sent to the Host.
- **Forwarding**: Packets are forwarded or switched from Port1 to Port2 using Slice0 (RX_PRU0 and TX_PRU0). Packets are forwarded from Port2 to Port1 using Slice1 (RX_PRU1 and TX_PRU1).
- **Local Injection**: Packets are sent from Host to Ports 1/2 via the RTU_PRU and TX_PRU.

 \imageStyle{icssg_packet_flow_paths.png,width:80%}
 \image html icssg_packet_flow_paths.png  **Figure3**: ICSSG packet flow paths

# Memory usage by ICSSG Queues 

The ICSSG Queues used for packet content buffering are elaborated here <br>

File : `{Any of ENET ICSSG examples} CCS Project > Generated Source > SysConfig > ti_enet_soc.c`

## Switch mode
Buffer are allocated for each of the packet flow paths

<table>
<tr>
    <th style="width:300px">Memory Pool</th>
    <th style="width:300px">Purpose</th>
    <th style="width:250px">Default size of each Queue in Pool</th>
    <th style="width:250px">Number of Queues</th>
    <th style="width:250px">Default Total pool size</th>
</tr>
<tr>
    <td>gEnetSoc_icssg1PortPoolMem_0
    <td>Forwarding path buffer used for the purpose of Forwarding of Frames from Port 1 to Port 2
    <td>6KB
    <td>Default: 8 <br>(Max: 8 to support 8 QoS)
    <td>48KB
</tr>
<tr>
    <td>gEnetSoc_icssg1PortPoolMem_1
    <td>Forwarding path buffer used for the purpose of Forwarding of Frames from Port 2 to Port 1
    <td>6KB
    <td>Default: 8 <br>(Max: 8 to support 8 QoS)
    <td>48KB
</tr>
<tr>
    <td>gEnetSoc_icssg1HostPoolMem_0
    <td>Local Injection path buffer used for the purpose of transmitting frames from the host to Port A or/and Port B via ICSSG Slice 0.
    <td>6KB when Gigabit support is enabled <br>3KB when Gigabit support is disabled 
    <td>Calculated as: n = 2 * QoS Level <br>(Factor 2 accounts for the Own-port Host pool and Opposite-port Host pool) <br><br>Default Configuration: QoS level is 3 by default in SysCfg, hence there are 2 * 3 = 6 queues 
    <td>36KB when Gigabit support is enabled <br>18KB when Gigabit support is disabled 
</tr>
<tr>
    <td>gEnetSoc_icssg1HostPoolMem_1
    <td>Local Injection path buffer used for the purpose of transmitting frames from the host to Port A or/and Port B via ICSSG Slice 1.
    <td>6KB when Gigabit support is enabled <br>3KB when Gigabit support is disabled 
    <td>Calculated as: n = 2 * QoS Level<br>(Factor 2 accounts for the Own-port Host pool and Opposite-port Host pool) <br><br>Default Configuration: QoS level is 3 by default in SysCfg, hence there are 2 * 3 = 6 queues 
    <td>36KB when Gigabit support is enabled <br>18KB when Gigabit support is disabled 
</tr>
<tr>
    <td>gEnetSoc_icssg1HostQueueMem_0
    <td>Host Egress path buffer used for the purpose of reception of frames from the Port A to the host.
    <td>8KB when Gigabit support is enabled <br>5KB when Gigabit support is disabled 
    <td>Default: 2
    <td>16KB when Gigabit support is enabled <br>10KB when Gigabit support is disabled
</tr>
<tr>
    <td>gEnetSoc_icssg1HostQueueMem_1
    <td>Host Egress path buffer used for the purpose of reception of frames from the Port B to the host.
    <td>8KB when Gigabit support is enabled <br>5KB when Gigabit support is disabled 
    <td>Default: 2
    <td>16KB when Gigabit support is enabled <br>10KB when Gigabit support is disabled
</tr>
</table>

## Dual-MAC mode
Buffers are allocated for the Host egress and Local injection paths. However, no buffers are allocated for forwarding path since Dual-MAC mode by definition does not have the feature of forwarding.

<table>
<tr>
    <th style="width:300px">Memory Pool</th>
    <th style="width:300px">Purpose</th>
    <th style="width:250px">Default size of each Queue in Pool</th>
    <th style="width:250px">Number of Queues</th>
    <th style="width:250px">Default Total pool size</th>
</tr>
<tr>
    <td>gEnetSoc_icssg1PortPoolMem_0
    <td>Forwarding path buffer used for the purpose of Forwarding of Frames from Port 1 to Port 2
    <td>0KB
    <td>0
    <td>0KB
</tr>
<tr>
    <td>gEnetSoc_icssg1PortPoolMem_1
    <td>Forwarding path buffer used for the purpose of Forwarding of Frames from Port 2 to Port 1
    <td>0KB
    <td>0
    <td>0KB
</tr>
<tr>
    <td>gEnetSoc_icssg1HostPoolMem_0
    <td>Local Injection path buffer used for the purpose of transmitting frames from the host to Port A or/and Port B via ICSSG Slice 0.
    <td>8KB when Gigabit support is enabled <br>4KB when Gigabit support is disabled 
    <td>n = QoS Level <br>Default: QoS level is 3 by default in SysCfg
    <td>24KB when Gigabit support is enabled <br>12KB when Gigabit support is disabled 
</tr>
<tr>
    <td>gEnetSoc_icssg1HostPoolMem_1
    <td>Local Injection path buffer used for the purpose of transmitting frames from the host to Port A or/and Port B via ICSSG Slice 1.
    <td>8KB when Gigabit support is enabled <br>4KB when Gigabit support is disabled 
    <td>n = QoS Level <br>Default: QoS level is 3 by default in SysCfg
    <td>24KB when Gigabit support is enabled <br>12KB when Gigabit support is disabled 
</tr>
<tr>
    <td>gEnetSoc_icssg1HostQueueMem_0
    <td>Host Egress path buffer used for the purpose of reception of frames from the Port A to the host.
    <td>8KB when Gigabit support is enabled <br>5KB when Gigabit support is disabled 
    <td>Default: 2
    <td>16KB when Gigabit support is enabled <br>10KB when Gigabit support is disabled
</tr>
<tr>
    <td>gEnetSoc_icssg1HostQueueMem_1
    <td>Host Egress path buffer used for the purpose of reception of frames from the Port B to the host.
    <td>8KB when Gigabit support is enabled <br>5KB when Gigabit support is disabled 
    <td>Default: 2
    <td>16KB when Gigabit support is enabled <br>10KB when Gigabit support is disabled
</tr>
</table>

# Modify Port Pool memory usage

Port pool memory usage can be modified by varying the:

## A. Size per queue and/or
- Go to file: `{MCU_SDK_install_path}\source\networking\enet\core\sysconfig\networking\.meta\enet_icss\templates\enet_soc_cfg_am64x_am243x.c.xdt`
- Modify the define `ICSSG_SWITCH_PORT_POOL_SIZE` to configure the required queue size.
- Save the file.
- Build the application.
- Now, you can see the sizes getting reflected in the generated file: `ti_enet_soc.c` (memory allocated can also be verified in the `.map` file under the `.icss_mem` output section)

## B. Number of Queues initialized

- ICSSG_SWITCH_PORT_BUFFER_POOL_NUM: The number of ICSSG Port buffer pools is by default defined as 8 (max) to provide a unique Pool for upto 8 QoS levels.
- This number can be reduced as per the requirement of the user application, for example 'n' (n = 1 to 8).
- However, care must be taken to ensure all priorities (PCPs) are mapped to only those 0 to n-1 Queues.
- This can be done by using the IOCTL `ENET_MACPORT_IOCTL_SET_EGRESS_QOS_PRI_MAP`

**Steps:**

- Go to file:  `{MCU_SDK_install_path}\source\networking\enet\core\include\per\icssg.h`
    - Change the `ICSSG_SWITCH_PORT_BUFFER_POOL_NUM` to any value from 1 to 8.
    - Save the file.
- Rebuild the library.
    - Run this command: `gmake -s -j8 -f makefile.am243x enet-icssg_r5f.ti-arm-clang` here `{MCU_SDK_install_path}`
- With respect to the the Enet ICSSG Layer 2 example `{MCU_SDK_install_path}\source\networking\enet\core\examples\enet_layer2_icssg\`
    - EnetMp_rxTask() → Modify the prioMap[] array which is passed as argument to set_priority_queue_mapping()

**Example**
- If setting `ICSSG_SWITCH_PORT_BUFFER_POOL_NUM = 3`, Then the available pools will only be 0 ,1, 2.
- Hence, all the traffic must be directed to only these queues 0 to 2 by using the above IOCTL. Type of mapping is left to the user.

The Input Argument of the `IOCTL ENET_MACPORT_IOCTL_SET_EGRESS_QOS_PRI_MAP`  can be set based on below:
uint32_t EnetPort_PriorityMap::priorityMap[ENET_PRI_NUM] : The array index corresponds to the PCP and the value holds the mapped Q value

<table>
<tr>
  <th>PCP(Index)
  <th>Queue Number = priorityMap[pcp]
<tr>
  <td>0
  <td>0
<tr>
  <td>1
  <td>0
<tr>
  <td>2
  <td>0
<tr>
  <td>3
  <td>1
<tr>
  <td>4
  <td>1
<tr>
  <td>5
  <td>2
<tr>
  <td>6
  <td>2
<tr>
  <td>7
  <td>2
</table>

Sample IOCTL usage: `ENET_MACPORT_IOCTL_SET_EGRESS_QOS_PRI_MAP`

```c
/* Mapping the PCP to Queue*/   
EnetMacPort_SetEgressPriorityMapInArgs SetEgressPriorityMapInArgs;
EnetPort_PriorityMap PriorityMap;          
PriorityMap.priorityMap[0] = 0;
PriorityMap.priorityMap[1] = 0;
PriorityMap.priorityMap[2] = 0;
PriorityMap.priorityMap[3] = 1;
PriorityMap.priorityMap[4] = 1;
PriorityMap.priorityMap[5] = 2;
PriorityMap.priorityMap[6] = 2;
PriorityMap.priorityMap[7] = 2;

SetEgressPriorityMapInArgs.macPort = macPortList[i];
SetEgressPriorityMapInArgs.priorityMap = PriorityMap;
ENET_IOCTL_SET_IN_ARGS(&prms, &SetEgressPriorityMapInArgs);
ENET_IOCTL(hEnet, coreId, ENET_MACPORT_IOCTL_SET_EGRESS_QOS_PRI_MAP, &prms, status);
if (status != ENET_SOK)
{
    EnetAppUtils_print("EnetApp_enablePorts() failed to set PCP to Q map: %d\r\n", status);
}
```

# Modify Host Pool memory usage

Host pool memory usage can be modified by varying the:

## A. Size per Queue (and/or)
- Go to file: `{MCU_SDK_install_path}\source\networking\enet\core\sysconfig\networking\.meta\enet_icss\templates\enet_soc_cfg_am64x_am243x.c.xdt`
- Modify the define `ICSSG_SWITCH_HOST_POOL_SIZE` to configure the required queue size.
- Save the file.
- Build the application.
- Now you can see the sizes getting reflected in the generated file: `ti_enet_soc.c`.
- Memory allocated can also be verified in the `.map` file under the `.icss_mem` output section.

## B. Number of Queues initialized
- `ICSSG_SWITCH_HOST_BUFFER_POOL_NUM_CFG`: The number of ICSSG Host buffer pools is by default defined as (2 * QoS Level). By default, QoS Level=3 hence number of queue in the pool is 6.

To modify the number of pools:

- Go to file: `{MCU_SDK_install_path}\source\networking\enet\core\sysconfig\networking\.meta\enet_icss\enet_icss.syscfg`
- Modify the QoS Level parameter to your desired value.
- Build the application.
- Now, you can see the sizes getting reflected in the generated file: `ti_enet_soc.c`. (Memory allocated can also be verified in the `.map` file under the `.icss_mem` output section)

## Locally inject different Traffic classes
According to IEEE 802.1D, there can be different traffic types like best effort, excellent effort generated by different user application. Some of these traffic might be latency sensitive for ex: PTP network control traffic.
Traffic-class to transmit queue mapping feature determines the transmit queue for transmitting data streams based on traffic class configured by the user application. Port queues supports maximum of 8 queues (per port) to treat the traffic at QoS level. For ex: best effort data can be segregated from all the time-critical traffic. Port queues are serviced from highest (7) to lowest priority (0), so highest `txPktTc` variable number (between 0-7) shall be serviced first.
There are a total of 8 QoS Levels and each QoS Level has a dedicated Queue.
Also, multiple traffic types can be combined to single queue. 
Refer IEEE 802.1D for more details.

- When the user wants to send out packets from the Host to either ports via the ICSSG, it can be sent via different one of the 8 Host Pool queues (`gEnetSoc_icssg1HostPoolMem_0/1`).
    - This can be achieved by using the `txPktInfo->txPktTc` parameter.
    - Refer to `enet_layer2_icssg.c`.
    - `txPktTc` can range from 0 to 7 corresponding to the 8 levels of QoS.
    
\note `txPktTc` passed should be less than the "QoS Level" set in SysCfg by the user. If not, the packets will be sent to a queue which has not been initialized itself, resulting in stall due to no buffer.


