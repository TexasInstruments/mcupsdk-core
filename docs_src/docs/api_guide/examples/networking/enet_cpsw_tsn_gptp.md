# Enet TSN gPTP Example {#EXAMPLES_ENET_CPSW_TSN_GPTP}

[TOC]

# Introduction
This Enet TSN example illustrates the usage of gPTP 802.1AS stack with CPSW peripheral. The application illustrates the following 
    - gPTP EndPoint slave mode
    - gPTP EndPoint master mode
    - gPTP Switch mode

Yang based configuration is also supported. Currently File System is not supported, will be added in future releases. In this example, We use two Rx and two Tx DMA channel, one Rx and one Tx channel specific to gPTP traffic. All the non-PTP traffic is handled on the other DMA channel in a echo task.

# Supported Combinations

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/networking/tsn/gptp_app

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/networking/tsn/gptp_app

\endcond
# Steps to Run the Example

## Build the example

- When using CCS projects to build, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- When using makefiles to build, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)

## HW Setup

\note Make sure you have setup the EVM with cable connections as shown here, \ref EVM_SETUP_PAGE.
      In addition do below steps.

\cond SOC_AM64X

### AM64X-EVM

#### For CPSW based example

- Connect a ethernet cable to the EVM from host PC as shown below. If you have multiple boards, you can connect both of them and run the same application on both the boards.

  \imageStyle{am64x_evm_lwip_example_00.png,width:30%}
  \image html am64x_evm_lwip_example_00.png Ethernet cable for CPSW based ethernet

\endcond

\cond SOC_AM243X

### AM243X-EVM

#### For CPSW based example

- Connect a ethernet cable to the EVM from host PC as shown below. If you have multiple boards, you can connect both of them and run the same application on both the boards.

  \imageStyle{am64x_evm_lwip_example_00.png,width:30%}
  \image html am64x_evm_lwip_example_00.png Ethernet cable for CPSW based ethernet

### AM243X-LP

\note AM243X-LP has two ethernet Ports which can be configured as both CPSW/ICSS ports.

#### For CPSW based examples

- Connect a ethernet cable to the EVM from host PC as shown below. If you have multiple boards, you can connect both of them and run the same application on both the boards.

  \imageStyle{am243x_lp_lwip_example_00.png,width:30%}
  \image html am243x_lp_lwip_example_00.png Ethernet cable for CPSW based ethernet

\endcond


## Run the example

\attention If you need to reload and run again, a CPU power-cycle is MUST

- Launch a CCS debug session and run the example executable, see \ref CCS_LAUNCH_PAGE
- You can either use two boards connected to each other and run the example on both of them. Or you can connect to a linux PC as other PTP instance. Please make sure your linux PC has a gPTP capable network card.
- If you are connecting the board to a linux PC. You should use the ptp4l gptp config https://github.com/richardcochran/linuxptp/blob/master/configs/gPTP.cfg on the linux.
- Command on Linux: 
\code
sudo ptp4l -i eno1 -m -l 6 -q -f ~/gptp_config.cfg
-i <interface name> -f <path to cfg file>
\endcode
- For some network cards, there is a bug with prop delay. So, you might need to increase the neighborPropDelayThresh in that case as below.
\code
[global]

gmCapable               1
priority1               248
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

- `priority1` can be changed to make the linux PC master or slave. Lower the number higher the priority to become master.
- You will see logs in the UART terminal as shown in the next section. The linux PC side logs are with intel i210 card.

## Sample output for Enet gPTP example

\cond SOC_AM243X

- Linux PC as gPTP slave and DUT as gPTP master

\code
==========================
          gPTP App        
==========================
Enabling clocks!
sitara-cpsw: Create RX task for regular traffic 
start to open driver.
EnetAppUtils_reduceCoreMacAllocation: Reduced Mac Address Allocation for CoreId:1 From 4 To 2 

Init all configs
----------------------------------------------
sitara-cpsw: init config
Mdio_open:294 
sitara-cpsw: Open port 1
EnetPhy_bindDriver:1717 
sitara-cpsw: Open port 2
EnetPhy_bindDriver:1717 
PHY 3 is alive
PHY 15 is alive
initQs() txFreePktInfoQ initialized with 16 pkts
MAC port addr: f4:84:4c:fb:c0:42
unibase-1.1.5
app_start:gptp start done!
INF:def04:simpledb_open:no data is imported
INF:def04:uc_hwal_open:
INF:cbase:cb_rawsock_open:dmaTxChId=0 dmaRxChId=0 nTxPkts=16 nRxPkts=32 pktSize=1536
INF:cbase:Mac addr has not been allocated
INF:def04:000000-082196:uniconf_main:uniconf started
gptp_task: started.
gptp_task: dbname=INF:def04:get_exmodid_in_db:first xl4gptp:exmodid=0
INF:gptp:gptpman_run:max_domains=1, max_ports=2
INF:cbase:cb_rawsock_open:dmaTxChId=1 dmaRxChId=1 nTxPkts=16 nRxPkts=32 pktSize=1536
INF:cbase:Mac addr has not been allocated
INF:gptp:dev:tilld0 open success
INF:gptp:dev:tilld1 open success
INF:gptp:gptpnet_init:Open lldtsync OK!
INF:gptp:IEEE1588-2019 performance monitoring enabled.
INF:gptp:onenet_activate:tilld0 status=0, duplex=0, speed=0Mbps
INF:gptp:onenet_activate:tilld1 status=0, duplex=0, speed=0Mbps
INF:gptp:000000-250071:domainIndex=0, GM changed old=00:00:00:00:00:00:00:00, new=F4:84:4C:FF:FE:FB:C0:42
INF:gptp:gptpclock_set_gmsync:gptpInstanceIndex=0, domainIndex=0, gmstate=2
INF:gptp:set_phase_offsetGM:domainIndex=0, New adjustment(New GM?)
Cpsw_handleLinkUp:1449 
MAC Port 1: link up
INF:gptp:index=1 speed=1000, duplex=full, ptpdev=tilld0
WRN:gptp:000004-750003:waiting_for_pdelay_interval_timer_proc:portIndex=1, sourcePortIdentity=68:05:CA:FF:FE:C8:7A:C2, thisClock=F4:84:4C:FF:FE:FB:C0:42, neighborPropDelay=201
INF:gptp:waiting_for_pdelay_interval_timer_proc:portIndex, not asCapable
INF:gptp:md_pdelay_resp_sm_recv_req:port=1, set receivedNonCMLDSPdelayReq=1
      5. 60s : CPU load =   5.52 %
INF:gptp:waiting_for_pdelay_interval_timer_proc:set asCapableAcrossDomains, portIndex=1
INF:gptp:set asCapable for domainIndex=0, portIndex=1
INF:gptp:000005-759829:gptpgcfg_set_asCapable:domainInde=0, portIndex=1, ascapable=1
INF:gptp:000005-768267:gm_stable:gm_unstable_proc:domainIndex=0
INF:gptp:gptpclock_set_gmsync:gptpInstanceIndex=0, domainIndex=0, gmstate=1
INF:gptp:000005-779851:gm_stable:gm_unstable_proc:domainIndex=0
INF:gptp:000005-874389:setSyncTwoStep_txSync:domainIndex=0, portIndex=1, sync gap=5874msec
INF:gptp:000005-879959:setFollowUp_txFollowUp:domainIndex=0, portIndex=1, fup gap=5879msec
INF:gptp:000006-875607:gm_stable:gm_stable_proc:domainIndex=0
INF:gptp:gptpclock_set_gmsync:gptpInstanceIndex=0, domainIndex=0, gmstate=2
domain=0, offset=0nsec, hw-adjrate=0ppb
        gmsync=true, last_setts64=0nsec
     10. 61s : CPU load =   4.36 %
     15. 62s : CPU load =   3.99 %
domain=0, offset=0nsec, hw-adjrate=0ppb
        gmsync=true, last_setts64=0nsec
     20. 63s : CPU load =   4.11 %
     25. 64s : CPU load =   3.97 %
domain=0, offset=0nsec, hw-adjrate=0ppb
        gmsync=true, last_setts64=0nsec
     30. 65s : CPU load =   4.11 %
     35. 66s : CPU load =   4.11 %
     40. 67s : CPU load =   4.01 %
\endcode

- PC gPTP slave logs

\code
ptp4l[8056.842]: selected /dev/ptp0 as PTP clock
ptp4l[8056.892]: port 1: INITIALIZING to LISTENING on INIT_COMPLETE
ptp4l[8056.892]: port 0: INITIALIZING to LISTENING on INIT_COMPLETE
ptp4l[8059.849]: port 1: link down
ptp4l[8059.849]: port 1: LISTENING to FAULTY on FAULT_DETECTED (FT_UNSPECIFIED)
ptp4l[8059.871]: selected local clock 6805ca.fffe.c87ac2 as best master
ptp4l[8059.871]: assuming the grand master role
ptp4l[8062.744]: port 1: link up
ptp4l[8062.784]: port 1: FAULTY to LISTENING on INIT_COMPLETE
ptp4l[8066.197]: port 1: LISTENING to MASTER on ANNOUNCE_RECEIPT_TIMEOUT_EXPIRES
ptp4l[8066.197]: selected local clock 6805ca.fffe.c87ac2 as best master
ptp4l[8066.197]: assuming the grand master role
ptp4l[8066.699]: port 1: new foreign master f4844c.fffe.fbc042-1
ptp4l[8068.699]: selected best master clock f4844c.fffe.fbc042
ptp4l[8068.699]: port 1: MASTER to UNCALIBRATED on RS_SLAVE
ptp4l[8069.078]: port 1: UNCALIBRATED to SLAVE on MASTER_CLOCK_SELECTED
ptp4l[8069.700]: rms 3879921009606 max 7759842019460 freq -34943 +/- 7727 delay   200 +/-   0
ptp4l[8070.700]: rms 2254 max 3442 freq -27102 +/- 3073 delay   199 +/-   0
ptp4l[8071.700]: rms 3762 max 3942 freq -20229 +/- 1002 delay   198 +/-   0
ptp4l[8072.700]: rms 2393 max 3174 freq -18880 +/-  93 delay   199 +/-   0
ptp4l[8073.700]: rms  776 max 1287 freq -19609 +/- 280 delay   198 +/-   0
ptp4l[8074.700]: rms  141 max  219 freq -20433 +/- 178 delay   198 +/-   0
ptp4l[8075.700]: rms  228 max  243 freq -20826 +/-  57 delay   198 +/-   0
ptp4l[8076.700]: rms  138 max  192 freq -20894 +/-   9 delay   199 +/-   0
ptp4l[8077.700]: rms   40 max   71 freq -20839 +/-  22 delay   199 +/-   0
ptp4l[8078.700]: rms   10 max   17 freq -20791 +/-  11 delay   199 +/-   0
ptp4l[8079.700]: rms   15 max   19 freq -20768 +/-   7 delay   199 +/-   0
ptp4l[8080.700]: rms   11 max   14 freq -20760 +/-   3 delay   199 +/-   0
ptp4l[8081.700]: rms    3 max    6 freq -20768 +/-   4 delay   199 +/-   0
ptp4l[8082.700]: rms    4 max    7 freq -20763 +/-   4 delay   199 +/-   0
ptp4l[8083.700]: rms    3 max    6 freq -20767 +/-   4 delay   200 +/-   0
ptp4l[8084.700]: rms    3 max    4 freq -20768 +/-   3 delay   200 +/-   0
ptp4l[8085.700]: rms    5 max    7 freq -20776 +/-   3 delay   201 +/-   0
ptp4l[8086.700]: rms    4 max    6 freq -20769 +/-   5 delay   200 +/-   0
ptp4l[8087.700]: rms    9 max   15 freq -20755 +/-   7 delay   199 +/-   0
ptp4l[8088.700]: rms    7 max   11 freq -20750 +/-   5 delay   200 +/-   0
ptp4l[8089.700]: rms    5 max    7 freq -20749 +/-   6 delay   200 +/-   0
ptp4l[8090.700]: rms    4 max    6 freq -20750 +/-   5 delay   199 +/-   0
ptp4l[8091.700]: rms   11 max   14 freq -20771 +/-   6 delay   199 +/-   0
\endcode

- Linux PC as master and DUT as slave

\code
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

- DUT logs

\code
==========================
          gPTP App        
==========================
Enabling clocks!
sitara-cpsw: Create RX task for regular traffic 
start to open driver.
EnetAppUtils_reduceCoreMacAllocation: Reduced Mac Address Allocation for CoreId:1 From 4 To 2 

Init all configs
----------------------------------------------
sitara-cpsw: init config
Mdio_open:294 
sitara-cpsw: Open port 1
EnetPhy_bindDriver:1717 
sitara-cpsw: Open port 2
EnetPhy_bindDriver:1717 
PHY 3 is alive
PHY 15 is alive
initQs() txFreePktInfoQ initialized with 16 pkts
MAC port addr: f4:84:4c:fb:c0:42
unibase-1.1.5
app_start:gptp start done!
INF:def04:simpledb_open:no data is imported
INF:def04:uc_hwal_open:
INF:cbase:cb_rawsock_open:dmaTxChId=0 dmaRxChId=0 nTxPkts=16 nRxPkts=32 pktSize=1536
INF:cbase:Mac addr has not been allocated
INF:def04:000000-082218:uniconf_main:uniconf started
gptp_task: started.
gptp_task: dbname=INF:def04:get_exmodid_in_db:first xl4gptp:exmodid=0
INF:gptp:gptpman_run:max_domains=1, max_ports=2
INF:cbase:cb_rawsock_open:dmaTxChId=1 dmaRxChId=1 nTxPkts=16 nRxPkts=32 pktSize=1536
INF:cbase:Mac addr has not been allocated
INF:gptp:dev:tilld0 open success
INF:gptp:dev:tilld1 open success
INF:gptp:gptpnet_init:Open lldtsync OK!
INF:gptp:IEEE1588-2019 performance monitoring enabled.
INF:gptp:onenet_activate:tilld0 status=0, duplex=0, speed=0Mbps
INF:gptp:onenet_activate:tilld1 status=0, duplex=0, speed=0Mbps
INF:gptp:000000-250075:domainIndex=0, GM changed old=00:00:00:00:00:00:00:00, new=F4:84:4C:FF:FE:FB:C0:42
INF:gptp:gptpclock_set_gmsync:gptpInstanceIndex=0, domainIndex=0, gmstate=2
INF:gptp:set_phase_offsetGM:domainIndex=0, New adjustment(New GM?)
Cpsw_handleLinkUp:1449 
MAC Port 1: link up
INF:gptp:index=1 speed=1000, duplex=full, ptpdev=tilld0
WRN:gptp:000003-754416:waiting_for_pdelay_resp_condition:sequenceId doesn't match, expected=19314, received=19313
WRN:gptp:000003-763396:waiting_for_pdelay_resp_condition:Discard this RESP and wait for the next one, this could be because of SM Reset. SeqId: expected=19314, received=19313
WRN:gptp:000003-778684:waiting_for_pdelay_resp_follow_up_condition:portIndex=1, sequenceId doesn't match, expected=19314, received=19313
WRN:gptp:000003-790751:waiting_for_pdelay_resp_follow_up_condition:portIndex=1, PdelayResp comes twice for the same PdelayReq, sequenceId=19314
INF:gptp:md_pdelay_resp_sm_recv_req:port=1, set receivedNonCMLDSPdelayReq=1
WRN:gptp:000004-749940:waiting_for_pdelay_interval_timer_proc:portIndex=1, sourcePortIdentity=68:05:CA:FF:FE:C8:7A:C2, thisClock=F4:84:4C:FF:FE:FB:C0:42, neighborPropDelay=202
INF:gptp:waiting_for_pdelay_interval_timer_proc:portIndex=1, not asCapable
      5. 60s : CPU load =   5.53 %
INF:gptp:waiting_for_pdelay_interval_timer_proc:set asCapableAcrossDomains, portIndex=1
INF:gptp:set asCapable for domainIndex=0, portIndex=1
INF:gptp:000005-759595:gptpgcfg_set_asCapable:domainInde=0, portIndex=1, ascapable=1
INF:gptp:000005-767807:gm_stable:gm_unstable_proc:domainIndex=0
INF:gptp:gptpclock_set_gmsync:gptpInstanceIndex=0, domainIndex=0, gmstate=1
INF:gptp:000005-779397:gm_stable:gm_unstable_proc:domainIndex=0
INF:gptp:000005-874392:setSyncTwoStep_txSync:domainIndex=0, portIndex=1, sync gap=5874msec
INF:gptp:000005-879954:setFollowUp_txFollowUp:domainIndex=0, portIndex=1, fup gap=5879msec
INF:gptp:000006-775705:domainIndex=0, GM changed old=F4:84:4C:FF:FE:FB:C0:42, new=68:05:CA:FF:FE:C8:7A:C2
INF:gptp:gptpclock_set_gmsync:gptpInstanceIndex=0, domainIndex=0, gmstate=0
INF:gptp:000006-789098:gm_stable:gm_stable_proc:domainIndex=0
INF:gptp:000006-794316:gm_stable:gm_unstable_proc:domainIndex=0
INF:gptp:gptpclock_set_gmsync:gptpInstanceIndex=0, domainIndex=0, gmstate=1
INF:gptp:set_phase_offsetGM:domainIndex=0, New adjustment(New GM?)
INF:gptp:set_phase_offsetGM:domainIndex=0, offset adjustment, diff=475406949
INF:gptp:set_phase_offsetGM:domainIndex=0, stable
INF:gptp:clock_master_sync_receive:computeGmRateRatio:domainIndex=0 unstable rate=1944ppb (timeleap_future)
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 3323ppb, GMdiff=13790nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 6646ppb, GMdiff=17036nsec
domain=0, offset=0nsec, hw-adjrate=6646ppb
        gmsync=true, last_setts64=0nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 9886ppb, GMdiff=19600nsec
     10. 61s : CPU load =   4.56 %
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 12988ppb, GMdiff=21508nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 15909ppb, GMdiff=22796nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 18605ppb, GMdiff=23491nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 21040ppb, GMdiff=23637nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 23197ppb, GMdiff=23292nsec
     15. 62s : CPU load =   4.47 %
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 25060ppb, GMdiff=22514nsec
INF:gptp:000016-874280:gm_stable:gm_stable_proc:domainIndex=0
INF:gptp:gptpclock_set_gmsync:gptpInstanceIndex=0, domainIndex=0, gmstate=2
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 26627ppb, GMdiff=21372nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 27891ppb, GMdiff=19915nsec
INF:gptp:clock_master_sync_receive:computeGmRateRatio:domainIndex=0 unstable rate=-856ppb (timeleap_past)
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 28854ppb, GMdiff=18199nsec
domain=0, offset=0nsec, hw-adjrate=28854ppb
        gmsync=true, last_setts64=0nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 29538ppb, GMdiff=16302nsec
     20. 63s : CPU load =   4.53 %
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 29938ppb, GMdiff=14249nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 30097ppb, GMdiff=12137nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 30025ppb, GMdiff=9991nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 29745ppb, GMdiff=7858nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 29293ppb, GMdiff=5792nsec
     25. 64s : CPU load =   4.40 %
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 28683ppb, GMdiff=3809nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 27958ppb, GMdiff=1963nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 27135ppb, GMdiff=262nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 26240ppb, GMdiff=-1277nsec
domain=0, offset=0nsec, hw-adjrate=26240ppb
        gmsync=true, last_setts64=0nsec

\endcode
\endcond

\cond SOC_AM64X
- Linux PC as gPTP slave and DUT as gPTP master

\code
==========================
          gPTP App        
==========================
Enabling clocks!
sitara-cpsw: Create RX task for regular traffic 
start to open driver.
EnetAppUtils_reduceCoreMacAllocation: Reduced Mac Address Allocation for CoreId:1 From 4 To 2 

Init all configs
----------------------------------------------
sitara-cpsw: init config
Mdio_open:294 
sitara-cpsw: Open port 1
EnetPhy_bindDriver:1717 
sitara-cpsw: Open port 2
EnetPhy_bindDriver:1717 
PHY 0 is alive
PHY 3 is alive
initQs() txFreePktInfoQ initialized with 16 pkts
MAC port addr: 34:08:e1:80:a9:36
unibase-1.1.5
app_start:gptp start done!
INF:def04:simpledb_open:no data is imported
INF:def04:uc_hwal_open:
INF:cbase:cb_rawsock_open:dmaTxChId=0 dmaRxChId=0 nTxPkts=16 nRxPkts=32 pktSize=1536
INF:cbase:Mac addr has not been allocated
INF:def04:000000-083173:uniconf_main:uniconf started
gptp_task: started.
gptp_task: dbname=INF:def04:get_exmodid_in_db:first xl4gptp:exmodid=0
INF:gptp:gptpman_run:max_domains=1, max_ports=2
INF:cbase:cb_rawsock_open:dmaTxChId=1 dmaRxChId=1 nTxPkts=16 nRxPkts=32 pktSize=1536
INF:cbase:Mac addr has not been allocated
INF:gptp:dev:tilld0 open success
INF:gptp:dev:tilld1 open success
INF:gptp:gptpnet_init:Open lldtsync OK!
INF:gptp:IEEE1588-2019 performance monitoring enabled.
INF:gptp:onenet_activate:tilld0 status=0, duplex=0, speed=0Mbps
INF:gptp:onenet_activate:tilld1 status=0, duplex=0, speed=0Mbps
INF:gptp:000000-250060:domainIndex=0, GM changed old=00:00:00:00:00:00:00:00, new=34:08:E1:FF:FE:80:A9:36
INF:gptp:gptpclock_set_gmsync:gptpInstanceIndex=0, domainIndex=0, gmstate=2
INF:gptp:set_phase_offsetGM:domainIndex=0, New adjustment(New GM?)
Cpsw_handleLinkUp:1449 
MAC Port 1: link up
INF:gptp:index=1 speed=1000, duplex=full, ptpdev=tilld0
INF:gptp:md_pdelay_resp_sm_recv_req:port=1, set receivedNonCMLDSPdelayReq=1
      5. 61s : CPU load =   5.54 %
WRN:gptp:000005-750522:waiting_for_pdelay_interval_timer_proc:portIndex=1, sourcePortIdentity=68:05:CA:FF:FE:C8:7A:C2, thisClock=34:08:E1:FF:FE:80:A9:36, neighborPropDelay=215
INF:gptp:waiting_for_pdelay_interval_timer_proc:portIndex=1, not asCapable
INF:gptp:waiting_for_pdelay_interval_timer_proc:set asCapableAcrossDomains, portIndex=1
INF:gptp:set asCapable for domainIndex=0, portIndex=1
INF:gptp:000006-759636:gptpgcfg_set_asCapable:domainInde=0, portIndex=1, ascapable=1
INF:gptp:000006-767874:gm_stable:gm_unstable_proc:domainIndex=0
INF:gptp:gptpclock_set_gmsync:gptpInstanceIndex=0, domainIndex=0, gmstate=1
INF:gptp:000006-779474:gm_stable:gm_unstable_proc:domainIndex=0
INF:gptp:000006-874385:setSyncTwoStep_txSync:domainIndex=0, portIndex=1, sync gap=6874msec
INF:gptp:000006-879953:setFollowUp_txFollowUp:domainIndex=0, portIndex=1, fup gap=6879msec
INF:gptp:000007-874661:gm_stable:gm_stable_proc:domainIndex=0
INF:gptp:gptpclock_set_gmsync:gptpInstanceIndex=0, domainIndex=0, gmstate=2
domain=0, offset=0nsec, hw-adjrate=0ppb
        gmsync=true, last_setts64=0nsec
     10. 63s : CPU load =   4.40 %
     15. 64s : CPU load =   4.01 %
     20. 65s : CPU load =   4.14 %
domain=0, offset=0nsec, hw-adjrate=0ppb
        gmsync=true, last_setts64=0nsec
     25. 66s : CPU load =   4.03 %
     30. 67s : CPU load =   4.13 %
domain=0, offset=0nsec, hw-adjrate=0ppb
        gmsync=true, last_setts64=0nsec
        
\endcode

- PC gPTP slave logs

\code
ptp4l[302.853]: selected /dev/ptp0 as PTP clock
ptp4l[302.924]: port 1: INITIALIZING to LISTENING on INIT_COMPLETE
ptp4l[302.924]: port 0: INITIALIZING to LISTENING on INIT_COMPLETE
ptp4l[306.578]: port 1: LISTENING to MASTER on ANNOUNCE_RECEIPT_TIMEOUT_EXPIRES
ptp4l[306.578]: selected local clock 6805ca.fffe.c87ac2 as best master
ptp4l[306.578]: assuming the grand master role
ptp4l[306.830]: port 1: new foreign master 3408e1.fffe.80a936-1
ptp4l[308.831]: selected best master clock 3408e1.fffe.80a936
ptp4l[308.831]: port 1: MASTER to UNCALIBRATED on RS_SLAVE
ptp4l[309.209]: port 1: UNCALIBRATED to SLAVE on MASTER_CLOCK_SELECTED
ptp4l[309.831]: rms 846011109112972288 max 1692022218225946112 freq -44595 +/- 20153 delay   201 +/-   0
ptp4l[310.831]: rms 3739 max 5721 freq -35541 +/- 5086 delay   201 +/-   0
ptp4l[311.830]: rms 6234 max 6535 freq -24176 +/- 1658 delay   200 +/-   0
ptp4l[312.831]: rms 3966 max 5256 freq -21940 +/- 152 delay   200 +/-   0
ptp4l[313.831]: rms 1297 max 2132 freq -23123 +/- 456 delay   199 +/-   0
ptp4l[314.830]: rms  220 max  340 freq -24465 +/- 287 delay   200 +/-   0
ptp4l[315.830]: rms  355 max  378 freq -25088 +/-  84 delay   199 +/-   0
ptp4l[316.830]: rms  211 max  285 freq -25184 +/-  16 delay   199 +/-   0
ptp4l[317.830]: rms   59 max  103 freq -25096 +/-  31 delay   199 +/-   0
ptp4l[318.830]: rms   27 max   38 freq -25002 +/-  22 delay   199 +/-   0
ptp4l[319.830]: rms   37 max   42 freq -24950 +/-  12 delay   199 +/-   0
ptp4l[320.830]: rms   28 max   35 freq -24928 +/-   6 delay   199 +/-   0
ptp4l[321.830]: rms   20 max   27 freq -24914 +/-   6 delay   200 +/-   0
ptp4l[322.830]: rms   13 max   17 freq -24906 +/-   5 delay   200 +/-   0
ptp4l[323.830]: rms   15 max   17 freq -24888 +/-   3 delay   200 +/-   0
ptp4l[324.830]: rms   13 max   17 freq -24877 +/-   6 delay   200 +/-   0
ptp4l[325.830]: rms   15 max   20 freq -24859 +/-   7 delay   201 +/-   0
ptp4l[326.830]: rms   18 max   20 freq -24839 +/-   7 delay   201 +/-   0
ptp4l[327.830]: rms   14 max   18 freq -24827 +/-   7 delay   201 +/-   0
\endcode

- Linux PC as gPTP master and DUT as gPTP slave

\code
ptp4l[480.714]: selected /dev/ptp0 as PTP clock
ptp4l[480.772]: port 1: INITIALIZING to LISTENING on INIT_COMPLETE
ptp4l[480.772]: port 0: INITIALIZING to LISTENING on INIT_COMPLETE
ptp4l[483.492]: port 1: link down
ptp4l[483.492]: port 1: LISTENING to FAULTY on FAULT_DETECTED (FT_UNSPECIFIED)
ptp4l[483.507]: selected local clock 6805ca.fffe.c87ac2 as best master
ptp4l[483.507]: assuming the grand master role
ptp4l[486.452]: port 1: link up
ptp4l[486.524]: port 1: FAULTY to LISTENING on INIT_COMPLETE
ptp4l[490.210]: port 1: LISTENING to MASTER on ANNOUNCE_RECEIPT_TIMEOUT_EXPIRES
ptp4l[490.211]: selected local clock 6805ca.fffe.c87ac2 as best master
ptp4l[490.211]: assuming the grand master role
\endcode

- DUT logs

\code
==========================
          gPTP App        
==========================
Enabling clocks!
sitara-cpsw: Create RX task for regular traffic 
start to open driver.
EnetAppUtils_reduceCoreMacAllocation: Reduced Mac Address Allocation for CoreId:1 From 4 To 2 

Init all configs
----------------------------------------------
sitara-cpsw: init config
Mdio_open:294 
sitara-cpsw: Open port 1
EnetPhy_bindDriver:1717 
sitara-cpsw: Open port 2
EnetPhy_bindDriver:1717 
PHY 0 is alive
PHY 3 is alive
initQs() txFreePktInfoQ initialized with 16 pkts
MAC port addr: 34:08:e1:80:a9:36
unibase-1.1.5
app_start:gptp start done!
INF:def04:simpledb_open:no data is imported
INF:def04:uc_hwal_open:
INF:cbase:cb_rawsock_open:dmaTxChId=0 dmaRxChId=0 nTxPkts=16 nRxPkts=32 pktSize=1536
INF:cbase:Mac addr has not been allocated
INF:def04:000000-083207:uniconf_main:uniconf started
gptp_task: started.
gptp_task: dbname=INF:def04:get_exmodid_in_db:first xl4gptp:exmodid=0
INF:gptp:gptpman_run:max_domains=1, max_ports=2
INF:cbase:cb_rawsock_open:dmaTxChId=1 dmaRxChId=1 nTxPkts=16 nRxPkts=32 pktSize=1536
INF:cbase:Mac addr has not been allocated
INF:gptp:dev:tilld0 open success
INF:gptp:dev:tilld1 open success
INF:gptp:gptpnet_init:Open lldtsync OK!
INF:gptp:IEEE1588-2019 performance monitoring enabled.
INF:gptp:onenet_activate:tilld0 status=0, duplex=0, speed=0Mbps
INF:gptp:onenet_activate:tilld1 status=0, duplex=0, speed=0Mbps
INF:gptp:000000-250054:domainIndex=0, GM changed old=00:00:00:00:00:00:00:00, new=34:08:E1:FF:FE:80:A9:36
INF:gptp:gptpclock_set_gmsync:gptpInstanceIndex=0, domainIndex=0, gmstate=2
INF:gptp:set_phase_offsetGM:domainIndex=0, New adjustment(New GM?)
Cpsw_handleLinkUp:1449 
MAC Port 1: link up
INF:gptp:index=1 speed=1000, duplex=full, ptpdev=tilld0
WRN:gptp:000003-535968:waiting_for_pdelay_resp_condition:sequenceId doesn't match, expected=6097, received=6096
WRN:gptp:000003-542863:waiting_for_pdelay_resp_condition:Discard this RESP and wait for the next one, this could be because of SM Reset. SeqId: expected=6097, received=6096
WRN:gptp:000003-557981:waiting_for_pdelay_resp_follow_up_condition:portIndex=1, sequenceId doesn't match, expected=6097, received=6096
WRN:gptp:000003-569875:waiting_for_pdelay_resp_follow_up_condition:portIndex=1, PdelayResp comes twice for the same PdelayReq, sequenceId=6097
WRN:gptp:000004-499985:waiting_for_pdelay_interval_timer_proc:portIndex=1, sourcePortIdentity=68:05:CA:FF:FE:C8:7A:C2, thisClock=34:08:E1:FF:FE:80:A9:36, neighborPropDelay=207
INF:gptp:waiting_for_pdelay_interval_timer_proc:portIndex=1, not asCapable
INF:gptp:md_pdelay_resp_sm_recv_req:port=1, set receivedNonCMLDSPdelayReq=1
      5. 61s : CPU load =   5.59 %
INF:gptp:waiting_for_pdelay_interval_timer_proc:set asCapableAcrossDomains, portIndex=1
INF:gptp:set asCapable for domainIndex=0, portIndex=1
INF:gptp:000005-509628:gptpgcfg_set_asCapable:domainInde=0, portIndex=1, ascapable=1
INF:gptp:000005-517856:gm_stable:gm_unstable_proc:domainIndex=0
INF:gptp:gptpclock_set_gmsync:gptpInstanceIndex=0, domainIndex=0, gmstate=1
INF:gptp:000005-529441:gm_stable:gm_unstable_proc:domainIndex=0
INF:gptp:000005-624384:setSyncTwoStep_txSync:domainIndex=0, portIndex=1, sync gap=5624msec
INF:gptp:000005-629953:setFollowUp_txFollowUp:domainIndex=0, portIndex=1, fup gap=5629msec
INF:gptp:000006-624596:gm_stable:gm_stable_proc:domainIndex=0
INF:gptp:gptpclock_set_gmsync:gptpInstanceIndex=0, domainIndex=0, gmstate=2
INF:gptp:000007-223644:domainIndex=0, GM changed old=34:08:E1:FF:FE:80:A9:36, new=68:05:CA:FF:FE:C8:7A:C2
INF:gptp:gptpclock_set_gmsync:gptpInstanceIndex=0, domainIndex=0, gmstate=0
INF:gptp:000007-237037:gm_stable:gm_unstable_proc:domainIndex=0
INF:gptp:gptpclock_set_gmsync:gptpInstanceIndex=0, domainIndex=0, gmstate=1
INF:gptp:set_phase_offsetGM:domainIndex=0, New adjustment(New GM?)
INF:gptp:set_phase_offsetGM:domainIndex=0, offset adjustment, diff=-779404313
INF:gptp:set_phase_offsetGM:domainIndex=0, stable
INF:gptp:clock_master_sync_receive:computeGmRateRatio:domainIndex=0 unstable rate=802ppb (timeleap_future)
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 1907ppb, GMdiff=11052nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 3750ppb, GMdiff=12284nsec
domain=0, offset=0nsec, hw-adjrate=3750ppb
        gmsync=true, last_setts64=0nsec
     10. 62s : CPU load =   4.59 %
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 5495ppb, GMdiff=13149nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 7126ppb, GMdiff=13678nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 8610ppb, GMdiff=13873nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 9945ppb, GMdiff=13785nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 11111ppb, GMdiff=13430nsec
     15. 63s : CPU load =   4.45 %
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 12110ppb, GMdiff=12856nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 12926ppb, GMdiff=12072nsec
INF:gptp:000017-249245:gm_stable:gm_stable_proc:domainIndex=0
INF:gptp:gptpclock_set_gmsync:gptpInstanceIndex=0, domainIndex=0, gmstate=2
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 13577ppb, GMdiff=11144nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 14061ppb, GMdiff=10091nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 14384ppb, GMdiff=8947nsec
     20. 64s : CPU load =   4.58 %
domain=0, offset=0nsec, hw-adjrate=14384ppb
        gmsync=true, last_setts64=0nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 14556ppb, GMdiff=7741nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 14591ppb, GMdiff=6507nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 14504ppb, GMdiff=5277nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 14307ppb, GMdiff=4068nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 14015ppb, GMdiff=2901nsec
     25. 65s : CPU load =   4.48 %
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 13649ppb, GMdiff=1805nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 13649ppb, GMdiff=1805nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 13217ppb, GMdiff=783nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 12739ppb, GMdiff=-148nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 12231ppb, GMdiff=-971nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 11702ppb, GMdiff=-1693nsec
     30. 66s : CPU load =   4.56 %
\endcode
\endcond

# TSN Stack
This guide is intended to enhance user's understanding of the TSN stack and provide
guidance on how to seamlessly integrate TSN modules into their own applications.

## TSN source modules

The TSN library is composed of the following source modules:

- tsn_unibase: Universal utility libraries that are platform-independent.
- tsn_combase: Communication utility libraries that provide support for functions
  like sockets, mutexes, and semaphores.
- tsn_uniconf: Universal configuration daemon for Yang, provides APIs for
  developing a client application which retreives/writes yang parameters from/to database
  and ask the uniconf for configurating HW.
- tsn_gptp: Implementation of the IEEE 802.1 AS gptp protocol.

## TSN Initialization

A reference of the TSN stack initialization can be found in ``<mcu_plus_sdk>/examples/networking/tsn/gptp_app/tsninit.c``.
Prior to any module calls, it is necessary to initialize the unibase library once.
This can be achieved by invoking the ``tsn_init()`` function.

## TSN Logging

By default, TSN logs are directed to STDOUT using the fwrite function:
``fwrite(str, 1, strlen(str), stdout);``
However, it is possible to customize the log output by setting the ``console_out``
callback during the initialization of the unibase library in the ``tsn_init()``
function.

When a callback function is assigned to ``console_out`` it takes a significant amount
of time to log to the output device, it is advisable to log to a buffer and utilize
another task to log from the buffer to the desired output device.  This approach
helps to prevent log delays that could adversely affect the gPTP task, such as
incorrect *Sync* and *FollowUp* intervals.  The option to log to a buffer is
supported when the ``TSN_USE_LOG_BUFFER`` macro is defined.

To enable specific log levels, you can modify the ub_log_initstr parameter in
the tsn_init() function. There are eight log levels available::

    UBL_NONE = 0
    UBL_FATAL = 1
    UBL_ERROR = 2
    UBL_WARN = 3
    UBL_INFO = 4
    UBL_INFOV = 5
    UBL_DEBUG = 6
    UBL_DEBUGV = 7

For example:
`init_para.ub_log_initstr = "4,ubase:45,cbase:45,gptp:55,uconf:45";`

In the above example, `uconf:45` signifies the following:

``4 (INFO)``: Log level for printing to the console.
``5 (INFOV)``: Log level for storing in internal memory (which can be dumped to a file).

Please note that enabling the DEBUG or DEBUGV level will result in a large number
of logs being generated. If the log buffer is used, it may overflow, causing some
logs to be lost.

The first '4' character in `"4,ubase...` represents the default log level applied
to all modules.

## Starting uniconf and gPTP

The gPTP functionality operates in a separate task, alongside the universal configuration daemon "uniconf". 
To start these tasks, use the ``tsn_app_start()`` in tsninit.c as reference .

This function will start the uniconf and gPTP tasks.

## TSN Deinitialization

To deinitialize the TSN modules, you can invoke the ``tsn_app_stop();`` and ``tsn_app_deinit();`` functions.

# Integration
## Source integration

To integrate the TSN stack into your application, follow these steps:

- Initialize Enet LLD and setup board dependencies.  In the TSN example application,
  the initialization routines can be found at ``<mcu_plus_sdk>/examples/networking/tsn/gptp_app/tsnapp_main.c``,
  which can be used as reference.

  The main functions related to EVM board initialization are::
\code
      EnetAppUtils_enableClocks();
      EnetApp_driverInit();
      EnetApp_driverOpen();
      ...
\endcode

  These functions are responsible for configuring pinmux, GPIOs, etc.,
  as well as providing the MAC port and PHY configuration parameters, which are
  board specific.

- Before calling any TSN module APIs, ensure you initialize TSN by calling
  ``tsn_app_init()``.

- Initialize the *combase* Ethernet device table as the following code snippet::

\code
      for (i = 0; i < numEthPorts; i++)
      {
          snprintf(&gEnetApp_gPtpNetDevs[i][0], IFNAMSIZ, "tilld%d", i);
          /* ethdevs[i].netdev must hold a global address, not a stack address */
          ethdevs[i].netdev = gEnetApp_gPtpNetDevs[i];
          ethdevs[i].macport = ethPorts[i].macPort;
      }

      cb_lld_init_devs_table(ethdevs, i, enetType, instId);
\endcode

  In the above code, the Ethernet device name is not mandatory but it's useful as
  a virtual Ethernet device table that facilitates the conversion between the device
  name and the MAC port number.

  Please ensure that the ``ethdevs[i].netdev`` pointer points to a global memory
  location rather than a stack memory. The *combase* layer will reference this
  address instead of making a copy.

- To provide syscfg values to the TSN stack, you can use the ``cb_socket_set_lldcfg_update_cb()```
  - The parameter for this function is a callback function that updates the cb_socket_lldcfg_update_t values.


- Start the gPTP task by calling ``tsn_app_start()``. This will start the necessary threads for gPTP and uniconf.

## Uniconf configuration

Universal configuration daemon for Yang, provides APIs for developing a client application which retreives/writes yang parameters from/to database and ask the uniconf for configurating HW.

Uniconf supports the following config paramaters:

```
typedef struct ucman_data {  
    const char *dbname;  
    const char **configfiles;  
    int numconfigfile;  
    uint8_t ucmode;
    bool *stoprun;  
    int rval;  
    UC_NOTICE_SIG_T *ucmanstart;  
    const char *hwmod;
} ucman_data_t;
```

- ``dbname``: Specify the path of database file in the file system.
If user wants to write all config parameters to database file to make the config parameter
persist after a reboot, the full path of database file must be set via this parameter.
Currently, as file system is not supported, this parameter is set to ``NULL``.

- ``configfiles``: array of runtime config files will be used by the uniconf. Run time config file is a
text file which has one-to-one mapping with yang files, the format of each line of a
run time config file is specified under ``<mcu_plus_sdk>/source/networking/tsn/tsn-stack/tsn_uniconf/README.org``.
Currently, as file system is not supported, this parameter is set to ``NULL``.

- ``numofconfigfile``: Specify how many config files have been set via the the ``configfiles`` array above.
Currently, this parameter is set to ``0``.

- ``ucmode``: This is for setting the expected mode of uniconf. The uniconf can only run in a separated thread, please set ``UC_CALLMODE_THREAD|UC_CALLMODE_UNICONF`` for this parameter.
    
- ``stoprun``: Address to a flag which signals the uniconf to stop in case the flag is true.

- ``rval``: Error code of the uniconf in case error occurs.

- ``ucmanstart``: The name of semaphore or a semaphore object in case of FreeRTOS. If this parameter
is not ``NULL``, the uniconf will post this semaphore to signal application layer indicating
that it has been started successfully.

- ``hwmod``: Configure the HW for the uniconf. If an empty string is specified (hwmod=""),
it means the uniconf will configure HW.
If "NONE" is specified (hwmod="NONE"), the uniconf will not configure HW when client
side writes the database and ask it for an update.

## gPTP Yang Parameters

This section describes the yang parameters used for gPTP.
These parameters are described in the file:
standard/ieee/draft/1588/ieee1588-ptp.yang
of the repository: https://github.com/YangModels/yang.git 

 - ``underlying-interface`` : 
    Named reference to a configured ptp port.

 - ``port-enable`` : 
    Enable or disable the port.  If the port is disabled, it will not be used for
    synchronization. 

 - ``log-announce-interval`` : 
    The logarithm to the base 2 of the announce interval in seconds. 
    The default value is 0, which corresponds to 1 second for the announce interval.

 - ``gptp-cap-receipt-timeout`` : 
    Number of transmission intervals that a port waits without
    receiving the gPTP capable message before it is determined to not run gPTP.

 - ``announce-receipt-timeout`` : 
    The number of times an announce interval must pass without receiving an announce
    message before the port is considered faulty.  The default value is 3. 

 - ``log-sync-interval`` :
    The logarithm to the base 2 of the sync interval in seconds. 
    The default value is -3, which corresponds to 125ms for the sync interval.
  
 - ``minor-version-number`` : 
    The minor version number of the gPTP protocol.  The default value is 1.

 - ``initial-log-announce-interval`` : 
    The logarithm to the base 2 of the initial announce interval. The default value is 0, which corresponds to 1 second for the announce interval. 

 - ``sync-receipt-timeout`` : 
    The number of sync intervals to must pass without receiving a sync message
    before the port is considered faulty.  The default value is 3.

 - ``initial-log-sync-interval`` : 
    The logarithm to the base 2 of the initial sync interval. 
    The default value is -3, which corresponds to 125ms for the sync interval.

 - ``current-log-sync-interval`` :
    The logarithm to the base 2 of the current sync interval. 
    The default value is -3, which corresponds to 125ms for the sync interval.

 - ``current-log-gptp-cap-interval`` : 
    The logarithm to the base 2 of the gPTP capable messages interval. The default value is 3, 
    which corresponds to 8 seconds.

 - ``initial-log-pdelay-req-interval`` : 
    The logarithm to the base 2 of the initial pdelay request interval. 
    The default value is 0, which corresponds to 1 second for the pdelay request.
  
 - ``current-log-pdelay-req-interval`` : 
    The logarithm to the base 2 of the current pdelay request interval. 
    The default value is 0, which corresponds to 1 second for the pdelay request.

 - ``allowed-lost-responses`` : 
    The number of consecutive pdelay request intervals with no pdelay response that 
    the port can miss before it is considered faulty.  The default value is 9.

 - ``allowed-faults`` : 
    The number of faults above with asCapable is set to false.  The default value is 9.

 - ``mean-link-delay-thresh`` : 
    The Propagation time threshold to consider that a linked port is not asCapable. 
    The default value is 0x27100000, which corresponds to 10000<<16, 10000nsec.

 - ``priority1`` : 
    The priority1 of the PTP instance, used in the best master clock algorithm. 
    The default value is 250. 

 - ``external-port-config-enable`` : 
    Enable or disable the external port configuration. This is set by default as false, 
    to have the port state determined by the gPTP protocol.

 - ``time-source`` : 
    The time source of the PTP instance.  The default value is "internal-oscillator".

 - ``ptp-timescale`` : 
    The PTP timescale of the PTP instance. The default value is true, having the timescale of 
    the Grandmaster PTP instance is PTP.


## gPTP configuration

There are following types of yang config parameters for gPTP:

  - Config parameters which belong to yang standard as described above, can be referred from  
  (github.com/YangModels/yang.git): ieee1588-ptp.yang, ieee802-dot1as-ptp.yang, etc. (gptp-yangconfig.xml in source/networking/tsn/tsn-stack/tsn_gptp/gptpconf/)

  - Non-yang config parameters which are parameters needed for the gPTP but they are  
  not defined in the yang standard. (gptp_nonyangconfig.xml in source/networking/tsn/tsn-stack/tsn_gptp/gptpconf/)

To configure gPTP, you can set/read the values of yang parameters during runtime by using the API \ref yang_db_runtime_put_oneline (see the tsninit.c), \ref yang_db_runtime_get_oneline for retrieving values and similarly for non-yang params these are the APIs `gptpgcfg_set_item()` to set, `gptpgcfg_get_item` to retreive, with prototype :
\code 
int gptpgcfg_set_item(uint8_t gptpInstanceIndex, uint8_t confitem,
                        bool status, void *value, uint32_t vsize);
\endcode
- ``gptpInstanceIndex`` : Index of the gptp to be configured. For RTOS, 
because only one instance is applicable, should be set to 0.

- ``confitem``: From the desired parameter in gptp_nonyangconfig.xml in source/networking/tsn/tsn-stack/tsn_gptp/gptpconf/ (Note: these xml files are only used for internal code generation), Add ``XL4_EXTMOD_XL4GPTP`` prefix to the param name with underscores as delimiter.
    - For instance, within the gptp_nonyangconfig.xml, ``USE_HW_PHASE_ADJUSTMENT`` will have an equivalent
    ``XL4_EXTMOD_XL4GPTP_USE_HW_PHASE_ADJUSTMENT``.
- ``status``: ``YDBI_CONFIG`` or ``YDBI_STATUS``.  ``YDBI_CONFIG`` is used
to configure the parameter at runtime.  ``YDBI_STATUS`` is used to
configure the parameter at runtime and store the value in the database
for persistence.
- ``value``: Pointer to the value to be set.
- ``vsize``: Size of the value to be set, for the items. 

Example usage:
``gptpgcfg_set_item(gpoptd.instnum, XL4_EXTMOD_XL4GPTP_USE_HW_PHASE_ADJUSTMENT,
  YDBI_CONFIG, &use_hwphase, sizeof(use_hwphase));``
- As a reference, please consult the ``<mcu_plus_sdk>/examples/networking/tsn/gptp_app/tsninit.c``
file.

- The configuration must be done before calling the ``gptpman_run()`` function
  inside the ``tsninit.c`` file, after the gptpgcfg_init function call.

### gPTP Performance optimization

Out of box configuration is not optimized for performance as the connected devices (like some intel cards) may not be compatible with these optimal configuration. The following configurations can be tuned for optimizing the gPTP performance:

 - ``XL4_EXTMOD_XL4GPTP_CLOCK_COMPUTE_INTERVAL_MSEC``: 
    The interval in milliseconds at which the gPTP clock is computed. 
    The default value is currently set to ``1000``, and has a parameter vsize of ``1`` (uint8_t).

 - ``XL4_EXTMOD_XL4GPTP_FREQ_OFFSET_UPDATE_MRATE_PPB``:
      The rate at which the frequency offset is updated. 
      The default value is currently set to ``10``, and has a parameter vsize of ``4`` (uint32_t).

 - ``XL4_EXTMOD_XL4GPTP_PHASE_OFFSET_IIR_ALPHA_STABLE_VALUE``:
      The reciprocal of this is the alpha value of the IIR filter used to compute the phase offset. The default value is currently set to ``10``, and has a parameter vsize of ``4`` (uint32_t).

 - ``XL4_EXTMOD_XL4GPTP_FREQ_OFFSET_IIR_ALPHA_STABLE_VALUE``:
      The reciprocal of this is the alpha value of the IIR filter used to compute the frequency offset. The default value is currently set to ``10``, and has a parameter vsize of ``4`` (uint32_t).

e.g. 
The following is an example configuration which you can use in the example to get better accuaracy if compatible with your devices in the setup. The following configuration is tested with intel i210 card and in between two EVMs.

```
uint8_t compute_interval = 100;
uint32_t freq_offset_update_mrate_ppb = 5;
uint32_t phase_offset_iir_alpha_stable_value = 1;
uint32_t freq_offset_iir_alpha_stable_value = 2;

gptpgcfg_set_item(gpoptd.instnum, XL4_EXTMOD_XL4GPTP_CLOCK_COMPUTE_INTERVAL_MSEC, YDBI_CONFIG, &compute_interval, sizeof(compute_interval));
gptpgcfg_set_item(gpoptd.instnum, XL4_EXTMOD_XL4GPTP_FREQ_OFFSET_UPDATE_MRATE_PPB, YDBI_CONFIG, &freq_offset_update_mrate_ppb, sizeof(freq_offset_update_mrate_ppb));
gptpgcfg_set_item(gpoptd.instnum, XL4_EXTMOD_XL4GPTP_PHASE_OFFSET_IIR_ALPHA_STABLE_VALUE, YDBI_CONFIG, &phase_offset_iir_alpha_stable_value, sizeof(phase_offset_iir_alpha_stable_value));
gptpgcfg_set_item(gpoptd.instnum, XL4_EXTMOD_XL4GPTP_FREQ_OFFSET_IIR_ALPHA_STABLE_VALUE, YDBI_CONFIG, &freq_offset_iir_alpha_stable_value, sizeof(freq_offset_iir_alpha_stable_value));
```
# Limitations
For all parameters related to interval, the minimum interval can be configured for gPTP is 125ms. The reason for that minimum interval is the implementation uses a base timer which has minimum period is 125ms. Thus, increasing the interval parameters like `sync-interval` which is not a multiple of 125ms may produce inaccurate interval of sending sync message.Or reducing interval of sync message below 125ms is not possible. And the same limitation applied for all remaining interval parameters.  

# See Also

\ref NETWORKING
