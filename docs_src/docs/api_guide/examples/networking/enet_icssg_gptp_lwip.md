# Ethernet TSN gPTP on ICSSG along with LwIP stack {#EXAMPLES_ENET_ICSSG_TSN_LWIP_GPTP}

[TOC]

# Introduction
This ethernet TSN example illustrates the usage of gPTP IEEE 802.1AS stack with ICSSG peripheral with LwIP network stack running in parallel. Here we demonstrate a simple TCP echo server using netconn APIs with the LwIP stack.
On the gPTP front, the application used here supports all the below modes:
    - gPTP End-Point time_transmitter mode (i.e. master mode)
    - gPTP End-Point time_receiver mode (i.e. slave mode)
    - gPTP Bridge mode

In this example, We use dedicated Rx and Tx DMA channels for gPTP traffic. Please note that only ICSSG switch mode is supported with gPTP.

Please refer to the \ref EXAMPLES_ENET_LWIP_ICSSG_TCPSERVER page for LwIP TCP echo server documentation.

See also :\ref ENET_CPSW_TSN_GPTP for gPTP stack documentation.


# Supported Combinations

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/networking/tsn/gptp_lwip_icssg

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/networking/tsn/gptp_lwip_icssg

\endcond

# Steps to Run the Example

## Prerequisites
-  EVM Board
- Cat6 ethernet cable
- PC with Linux Ubuntu OS (or any PC running bash shell) and PTP capable network card
- Install `linuxptp`
\code
$ sudo apt-get install linuxptp
$ ptp4l -v
\endcode
- Configure linuxptp
\code
$ wget https://raw.githubusercontent.com/richardcochran/linuxptp/v4.3/configs/gPTP.cfg -O ~/gptp_config.cfg
\endcode
\attention Change the value of `priority1` in ~/gptp_config.cfg file to `100`, to enforce it to gPTP master

\attention For some network cards, there is a bug with internal propagation delay. So, in those cases you might need to increase the `neighborPropDelayThresh` in ptp_config.cfg as below-

\code
$ cat ~/gptp_config.cfg
\#
\# 802.1AS example configuration containing those attributes which
\# differ from the defaults.  See the file, default.cfg, for the
/# complete list of available options.
/#
[global]
gmCapable               1
priority1               100
priority2               248
logAnnounceInterval     0
logSyncInterval         -3
syncReceiptTimeout      3
neighborPropDelayThresh 10000
min_neighbor_prop_delay -20000000
assume_two_step         1
path_trace_enabled      1
follow_up_info          1
transportSpecific       0x1
ptp_dst_mac             01:80:C2:00:00:0E
network_transport       L2
delay_mechanism         P2P

\endcode
- `priority1` in ~/gptp_config.cfg file can be changed to make the Linux PC master or slave. Lower the number higher the priority to become master.

Please refer to the \ref EXAMPLES_ENET_LWIP_ICSSG_TCPSERVER page to start a TCP client on PC.

## To Configure Static IP
Please refer to \ref NETWORKING_LWIP_STATIC_IP.

## Build the example

- When using CCS projects to build, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- When using makefiles to build, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
## HW Setup

\note Make sure you have setup the EVM with cable connections as shown here, \htmllink{@VAR_MCU_SDK_DOCS_PATH/EVM_SETUP_PAGE.html, EVM Setup}.
      In addition do below steps.

\cond SOC_AM64X

### AM64X-EVM

#### For ICSS based example

- Connect a ethernet cable to the EVM from host PC as shown below

  \imageStyle{am64x_evm_lwip_example_01.png,width:30%}
  \image html am64x_evm_lwip_example_01.png Ethernet cable for ICSS based ethernet

\endcond

\cond SOC_AM243X

### AM243X-EVM

#### For ICSS based example

- Connect a ethernet cable to the EVM from host PC as shown below

  \imageStyle{am64x_evm_lwip_example_01.png,width:30%}
  \image html am64x_evm_lwip_example_01.png Ethernet cable for ICSS based ethernet

### AM243X-LP

\note AM243X-LP has two ethernet Ports which can be configured as ICSS ports.

#### For ICSS based examples

- Connect a ethernet cable to the AM243X-LP from host PC as shown below

  \imageStyle{am243x_lp_lwip_example_00.png,width:30%}
  \image html am243x_lp_lwip_example_00.png Ethernet cable for ICSS based ethernet

\endcond

## Create a network between EVM and host PC
EVM and PC has to connected directly as shown below using CAT6 or CAT5 cable. If there is ethernet switch placed in between, make sure the switch is gPTP capable. Connect both EVM and PC to a DHCP server to get the IP addressess. 
  \imageStyle{Lwip-gPTP-setup.png,width:30%}
  \image html Lwip-gPTP-setup.png Local network between PC and EVM
 
PORT1 instead of PORT0 on EVM can be used as well.

## Run the example
  
\attention If you need to reload and run again, a CPU power-cycle is MUST

- Execute the below command in PC terminal to start the gPTP on linux: 
\code
$ sudo ptp4l -i eno1 -m -l 6 -q -f ~/gptp_config.cfg
\endcode
Replace eno1 with the network interface connected to your PC.

- After the evm gets the IP address, you can ping the IP address to verify the connection.
- Execute the below command in PC terminal to start a TCP communication with the EVM.
\code
ncat <ip-address-of-evm> 8888
\endcode
- You will see logs in the UART terminal as shown in the next section. PC side logs are with Intel i210 card.
- Launch a CCS debug session and run the example executable, see \ref CCS_LAUNCH_PAGE
- Connect board and PC as mentioned in "HW Setup" above.

## Sample Log Output
### PC Output
\code
$ sudo ptp4l -i eno1 -m -l 6 -q -f ~/gptp_config.cfg
ptp4l[7727.045]: selected /dev/ptp0 as PTP clock
ptp4l[7727.088]: port 1: INITIALIZING to LISTENING on INIT_COMPLETE
ptp4l[7727.088]: port 0: INITIALIZING to LISTENING on INIT_COMPLETE
ptp4l[7727.088]: port 1: link down
ptp4l[7727.088]: port 1: LISTENING to FAULTY on FAULT_DETECTED (FT_UNSPECIFIED)
ptp4l[7727.127]: selected local clock 6805ca.fffe.c87ac2 as best master
ptp4l[7727.127]: assuming the grand master role
ptp4l[7729.260]: port 1: link up
ptp4l[7729.300]: port 1: FAULTY to LISTENING on INIT_COMPLETE
ptp4l[7732.330]: port 1: new foreign master f4844c.fffe.fbc042-1
ptp4l[7732.354]: port 1: LISTENING to MASTER on ANNOUNCE_RECEIPT_TIMEOUT_EXPIRES
ptp4l[7732.354]: selected local clock 6805ca.fffe.c87ac2 as best master
ptp4l[7732.354]: assuming the grand master role
\endcode
### DUT output
\code
===============================
ICSSG GPTP LWIP TCP ECHO SERVER
================================
Enabling clocks!
Mdio_open:294 
EnetPhy_bindDriver:1828 
EnetPhy_bindDriver:1828 
PHY 3 is alive
PHY 15 is alive
unibase-1.1.4
INF:cbase:tilld0: has mac: F4:84:4C:FB:C0:42
INF:cbase:tilld1: has mac: 00:00:00:00:00:00
Start: uniconf_task
EnetApp_uniconfTask: dbname: NULL
INF:uconf:simpledb_open:no data is imported
INF:uconf:uc_hwal_open:
INF:cbase:cb_rawsock_open:combase-1.1.3
INF:cbase:cb_rawsock_open:dmaTxChId=-1 numRxChannels=0 dmaRxChId=-1 nTxPkts=0 nRxPkts=0 pktSize=0
INF:uconf:create_semname_with_dbname:null dbname is specified.
INF:cbase:cb_lld_task_create:alloc stack size=16384
INF:uconf:000000-191526:uniconf_main:uniconf started
EnetApp_gptpYangConfig:domain=0
INF:uconf:get_exmodid_in_db:first xl4gptp:exmodid=0
INF:uconf:create_semname_with_dbname:null dbname is specified.
EnetApp_gptpNonYangConfig:XL4_EXTMOD_XL4GPTP_SINGLE_CLOCK_MODE=1
EnetApp_gptpNonYangConfig:XL4_EXTMOD_XL4GPTP_USE_HW_PHASE_ADJUSTMENT=1
EnetApp_gptpNonYangConfig:XL4_EXTMOD_XL4GPTP_CLOCK_COMPUTE_INTERVAL_MSEC=100
EnetApp_gptpNonYangConfig:XL4_EXTMOD_XL4GPTP_FREQ_OFFSET_IIR_ALPHA_START_VALUE=1
EnetApp_gptpNonYangConfig:XL4_EXTMOD_XL4GPTP_FREQ_OFFSET_IIR_ALPHA_STABLE_VALUE=4
EnetApp_gptpNonYangConfig:XL4_EXTMOD_XL4GPTP_PHASE_OFFSET_IIR_ALPHA_START_VALUE=1
EnetApp_gptpNonYangConfig:XL4_EXTMOD_XL4GPTP_PHASE_OFFSET_IIR_ALPHA_STABLE_VALUE=4
EnetApp_gptpNonYangConfig:XL4_EXTMOD_XL4GPTP_MAX_DOMAIN_NUMBER=1
Start: gptp2d_task
EnetApp_initTsn:TSN app start done!
Starting lwIP, local interface IP is dhcp-enabled
[LWIPIF_LWIP] NETIF INIT SUCCESS
Host MAC address-0 : f4:84:4c:fb:c0:42
[LWIPIF_LWIP] Enet has been started successfully
[0]Enet IF UP Event. Local interface IP:0.0.0.0
INF:gptp:gptpman_run:max_domains=1, max_ports=2
INF:cbase:cb_rawsock_open:combase-1.1.3
INF:cbase:cb_rawsock_open:dmaTxChId=1 numRxChannels=2 dmaRxChId=1 nTxPkts=8 nRxPkts=8 pktSize=1536
INF:cbase:rxChId 1 has owner dmaRxShared 0
INF:cbase:rxChId 3 has owner dmaRxShared 0
For ICSSG, EthType and VlanId are not used to match the packet only dest addr is used 
For ICSSG, EthType and VlanId are not used to match the packet only dest addr is used 
INF:cbase:cbl_query_response:tilld0 link DOWN !!!!
[0]Waiting for network UP ...
INF:gptp:dev:tilld0 open success
INF:cbase:tilld1: alloc mac: 70:FF:76:1E:3A:07
INF:gptp:dev:tilld1 open success
INF:gptp:gptpnet_init:Open lldtsync OK!
INF:cbase:cbl_query_response:tilld1 link DOWN !!!!
INF:gptp:IEEE1588-2019 performance monitoring disabled.
INF:uconf:ydbi_get_item_ifk3vk0:no data
INF:uconf:ydbi_get_item_ifk3vk0:no data
INF:gptp:onenet_activate:tilld0 status=0, duplex=1, speed=0Mbps
INF:uconf:ydbi_get_item_ifk3vk0:no data
INF:uconf:ydbi_get_item_ifk3vk0:no data
INF:gptp:onenet_activate:tilld1 status=0, duplex=1, speed=0Mbps
INF:ubase:GPTP_MEDIUM_ALLOC: fragsize=16 fragused/fragnum=830/921 (90%)
INF:ubase:GPTP_SMALL_ALLOC: fragsize=4 fragused/fragnum=19/71 (26%)
INF:ubase:SM_DATA_INST: fragsize=8 fragused/fragnum=2002/2002 (100%)
INF:gptp:gptpman_run:GPTPNET_INTERVAL_TIMEOUT_NSEC=125000000
WRN:gptp:gptpgcfg_link_check:can't read speed

INF:gptp:000000-504656:domainIndex=0, GM changed old=00:00:00:00:00:00:00:00, new=F4:84:4C:FF:FE:FB:C0:42
INF:gptp:gptpclock_set_gmsync:gptpInstanceIndex=0, domainIndex=0, gmstate=2
INF:gptp:set_phase_offsetGM:domainIndex=0, New adjustment(New GM?)
[0]Waiting for network UP ...
Icssg_handleLinkUp:2594 
INF:cbase:cbl_query_response:tilld0: link UP, speed=1000, duplex=1 !!!!
INF:gptp:index=1 speed=1000, duplex=full, ptpdev=tilld0
Icssg_handleLinkUp:2594 
[0]Network Link UP Event
INF:cbase:cbl_query_response:tilld1: link UP, speed=1000, duplex=1 !!!!
INF:gptp:index=2 speed=1000, duplex=full, ptpdev=tilld1
WRN:gptp:000003-876633:waiting_for_pdelay_interval_timer_proc:portIndex=2, sourcePortIdentity=68:05:CA:FF:FE:C8:7A:C2, thisClock=F4:84:4C:FF:FE:FB:C0:42, neighborPropDelay=221
INF:gptp:waiting_for_pdelay_interval_timer_proc:portIndex=2, not asCapable
ERR:gptp:provide_rxframe: RX not ETH_P_1588 packet 0x88CC
[0]Waiting for network UP ...
INF:gptp:md_pdelay_resp_sm_recv_req:port=2, set receivedNonCMLDSPdelayReq=1
INF:gptp:waiting_for_pdelay_interval_timer_proc:set asCapableAcrossDomains, portIndex=2
INF:gptp:set asCapable for domainIndex=0, portIndex=2
INF:gptp:000004-884133:gptpgcfg_set_asCapable:domainInde=0, portIndex=2, ascapable=1
INF:gptp:000004-891519:gm_stable:gm_unstable_proc:domainIndex=0
INF:gptp:gptpclock_set_gmsync:gptpInstanceIndex=0, domainIndex=0, gmstate=1
INF:gptp:000004-903700:gm_stable:gm_unstable_proc:domainIndex=0
INF:gptp:000004-999147:setSyncTwoStep_txSync:domainIndex=0, portIndex=2, sync gap=5000msec
INF:gptp:000005-004383:setFollowUp_txFollowUp:domainIndex=0, portIndex=2, fup gap=5007msec
INF:gptp:000005-999187:gm_stable:gm_stable_proc:domainIndex=0
INF:gptp:gptpclock_set_gmsync:gptpInstanceIndex=0, domainIndex=0, gmstate=2
[0]Waiting for network UP ...
INF:gptp:000007-192911:domainIndex=0, GM changed old=F4:84:4C:FF:FE:FB:C0:42, new=68:05:CA:FF:FE:C8:7A:C2
INF:gptp:gptpclock_set_gmsync:gptpInstanceIndex=0, domainIndex=0, gmstate=0
INF:gptp:000007-206027:gm_stable:gm_unstable_proc:domainIndex=0
INF:gptp:gptpclock_set_gmsync:gptpInstanceIndex=0, domainIndex=0, gmstate=1
INF:gptp:set_phase_offsetGM:domainIndex=0, New adjustment(New GM?)
INF:gptp:set_phase_offsetGM:domainIndex=0, offset adjustment, diff=-50065501
INF:gptp:set_phase_offsetGM:domainIndex=0, offset adjustment by Freq., diff=12765
INF:gptp:set_phase_offsetGM:domainIndex=0, offset adjustment by Freq., diff=12871
INF:gptp:clock_master_sync_receive:computeGmRateRatio:domainIndex=0 unstable rate=847ppb (timeleap_future)
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 13718ppb, GMdiff=12871nsec
INF:gptp:set_phase_offsetGM:domainIndex=0, offset adjustment by Freq., diff=11458
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 13886ppb, GMdiff=11458nsec
INF:gptp:set_phase_offsetGM:domainIndex=0, stable
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 10660ppb, GMdiff=9825nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 2968ppb, GMdiff=8594nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 2881ppb, GMdiff=8323nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 2901ppb, GMdiff=8073nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 2784ppb, GMdiff=7814nsec
[0]Waiting for network UP ...
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 2668ppb, GMdiff=7333nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 2605ppb, GMdiff=7103nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 2584ppb, GMdiff=6885nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 2560ppb, GMdiff=6673nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 2496ppb, GMdiff=6463nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 2429ppb, GMdiff=6259nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 2320ppb, GMdiff=6056nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 2333ppb, GMdiff=5874nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 2316ppb, GMdiff=5694nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 2240ppb, GMdiff=5512nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 2208ppb, GMdiff=5341nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 2167ppb, GMdiff=5174nsec
domain=0, offset=0nsec, hw-adjrate=2167ppb
        gmsync=true, last_setts64=0nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 2125ppb, GMdiff=5012nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 2058ppb, GMdiff=4852nsec
[0]Waiting for network UP ...
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 1998ppb, GMdiff=4556nsec
[0]Enet IF UP Event. Local interface IP:172.24.227.210
\endcode

# See Also

\ref NETWORKING |
\ref EXAMPLES_ENET_CPSW_TSN_GPTP_BRIDGE |
\ref EXAMPLES_ENET_CPSW_TSN_GPTP_TR |
\ref EXAMPLES_ENET_CPSW_TSN_GPTP_TT |
\ref ENET_CPSW_TSN_GPTP