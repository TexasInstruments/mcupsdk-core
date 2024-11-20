Enet Integration Guide {#enet_integration_guide_top}
=====================

[TOC]

# Introduction {#cpsw_integration_guide_intro}
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

Enet LLD is an unified Ethernet driver that support Ethernet peripherals found
in TI SoCs, such as CPSW and ICSSG.  Please refer to the SDK release notes to
find out what peripherals are currently supported.

The diagram below shows the overall software architecture of the Enet low-level
driver.  A top-level driver layer provides the interface that the applications
can use to configure the switch and to send/receive Ethernet frames.

For instance, the CPSW support in the Enet driver consists of several software
submodules that mirror those of the CPSW hardware, like DMA, ALE, MAC port,
host port, MDIO, etc. Additionally, the Enet driver also includes PHY driver
support as well as a resource manager to administrate the CPSW resources.

Enet LLD relies on other drivers like UDMA for data transfer to/from the
Ethernet peripheral's host port to the other processing cores inside the TI
SoC devices. For the lower level access to the hardware registers, Enet LLD
relies on the Chip Support Library (CSL).

![Enet LLD Software Architecture Block Diagram](EnetLLD_Diagram.png "Enet LLD Software Architecture Block Diagram")

[Back To Top](@ref enet_integration_guide_top)




[Back To Top](@ref enet_integration_guide_top)


# Getting Familiar with Enet LLD APIs {#GettingFamiliarWithAPIs}
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

The Enet LLD APIs can be broadly divided into two categories: control and data
path.  The control APIs can be used to configure all Ethernet hardware submodules
like FDB, MAC port, host port, MDIO, statistics, as well as PHY drivers and
resource management.  The data path APIs are exclusive for the DMA-based
data transfers between the TI SoC processing cores and the Ethernet peripheral.

The main APIs of the Enet LLD are the following:

* Enet_open()
* Enet_close()
* Enet_ioctl()
* Enet_poll()
* Enet_periodicTick()
* EnetDma_openRxCh()
* EnetDma_closeRxCh()
* EnetDma_openTxCh()
* EnetDma_closeTxCh()
* EnetDma_retrieveRxPktQ()
* EnetDma_submitRxPktQ()
* EnetDma_retrieveTxPktQ()
* EnetDma_submitTxPktQ()

It's worth noting that the control path APIs are mainly IOCTL-based, and the
data path APIs are direct functions in order to avoid any additional overhead
associated with IOCTL calls as DMA data operations occur highly frequently.


## IOCTL Interface {#IOCTL_description}
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

IOCTLs are system calls that take an argument specifying the command code and
can take none or additional parameters via \ref Enet_IoctlPrms argument.
IOCTL are used by all Enet submodules except for DMA.

The \ref Enet_IoctlPrms parameter structure consists of input and output
argument pointers and their corresponding size.  The following helper macros are
provided to help construct the IOCTL params:

- ENET_IOCTL_SET_NO_ARGS(prms). Used for IOCTL commands that take no parameters.
- ENET_IOCTL_SET_IN_ARGS(prms, in). Used for IOCTL commands that take input
  parameters but don't output any parameter.
- ENET_IOCTL_SET_OUT_ARGS(prms, out). Used for IOCTL commands that don't take
  input parameters but return output parameters.
- ENET_IOCTL_SET_INOUT_ARGS(prms, in, out). Used for IOCTL commands that take
  input parameters and also return output parameters.

where `prms` in a pointer to \ref Enet_IoctlPrms variable, `in` is the pointer
to IOCTL input argument and `out` is the pointer to IOCTL output argument.

It's recommended that the application doesn't set the \ref Enet_IoctlPrms
members individually, but only through the helper macros listed above.

Please refer to the individual IOCTL command to find out if it requires input
and/or output parameters.

[Back To Top](@ref enet_integration_guide_top)


# Integrating Enet LLD into User's Application {#enet_integration_in_app}
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

Developers who wish to add network connectivity to the applications running on
TI SoCs, will have to integrate Enet LLD by following the below sequence:

-# @ref enet_init_sequence. One-time initialization before using any Enet LLD
   APIs.

-# @ref enet_open_sequence. Opens an Ethernet peripheral, it's called for
   each peripheral that the application intends to use, i.e. ICSSG0 and CPSW.

-# @ref enet_openport_sequence. Opens a MAC port.  For peripherals that
   provide multiple MAC port (i.e. CPSW9G that has 8 MAC ports), this sequence
   must be followed for each MAC port.  This section also covers MAC-PHY
   and MAC-MAC links.

-# @ref enet_pktrxtx_sequence. DMA TX and RX channel open sequences as well
   as functions to use to submit and retrieve queue of packets to/from the
   driver.

\if (am64x)
-# @ref enet_pktts_sequence. TX and RX packet timestamping sequence to
   enable timestamping and retrieval mechanism.
\endif

-# @ref enet_closeport_sequence. Closes a MAC port. Application has to close
   all MAC ports which were opened in step 3 when no longer needs those ports.

-# @ref enet_close_sequence. Closes an Ethernet peripheral when the application
   no longer needs it.  This sequence must be followed for each peripheral
   that was opened in step 2.

-# @ref enet_deinit_sequence. One-time deinitialization once application is
   done using Enet LLD.

Each of these sequences will be covered in detail in the following sections.

## Init Sequence {#enet_init_sequence}

This is a one-time initialization where the application sets the OSAL and utils
functions that Enet LLD will use throughout its lifecycle.

At this stage Enet LLD also initializes its SoC layer which contains data about
the Ethernet hardware available in the TI device.

The application should follow the next steps:

-# (Optional) \ref Enet_initOsalCfg() to initialize the OSAL configuration
   (see \ref EnetOsal_Cfg) with a default implementation.  The default
   implementation of Enet OSAL interface is based on OSAL layer.
   Typically, applications can directly use the default Enet OSAL implementation,
   unless Enet is being used on another OS (i.e. QNX) of if OS is not supported
   by OSAL layer.

-# (Optional) \ref Enet_initUtilsCfg() to initialize utils configuration
   (see \ref EnetUtils_Cfg) with a default implementation.  The default
   implementation of Enet Utils interface is UART-based logging and one-to-one
   address translations (physical-to-virtual and virtual-to-physical).
   Applications may want to pass their own utils' print function if UART is not
   available. Similarly, if a MMU is present and used, applications may need
   to pass their own address translation functions corresponding to the
   MMU configuration.

        utilsCfg.print = UART_printf;
        utilsCfg.physToVirt = &myPhysToVirtFxn;
        utilsCfg.virtToPhys = &myVirtToPhysFxn;

-# \ref Enet_init() to pass the OSAL and utils configurations setup in the
   previous two steps.  The application can pass NULL to either the OSAL config
   or utils config if it intends to use the driver's default implementation.


## Peripheral Open Sequence {#enet_open_sequence}

This is an initialization that needs to be done for each peripheral in the system.
The configuration parameters described in this section are peripheral specific.

Application should follow the next steps:

-# Initialize the peripheral configuration parameters with default values using
   \ref Enet_initCfg().  Although the application can fill all parameters manually,
   it's recommended tofirst get the driver's default values for all parameters and
   only overwrite the parameters of interest.

    - CPSW peripheral configuration - \ref Cpsw_Cfg structure has the configuration
      parameters for CPSW Enet Peripheral, such as:

       - Configuration of DMA: \ref EnetDma_Cfg.
       - VLAN configuration (inner/outer VLAN, customer/service switch):
         \ref Cpsw_VlanCfg.
       - Max packet length transmitted on egress: \ref Cpsw_Cfg.txMtu.
       - Configuration of the host (CPPI) port: \ref CpswHostPort_Cfg.
       - Configure of the ALE submodule: \ref CpswAle_Cfg.
       - Configure of the CPTS submodule: \ref CpswCpts_Cfg.
       - Configuration of the MDIO submodule: \ref Mdio_Cfg.
       - Configuration of CPSW Resource Partition: \ref EnetRm_ResCfg.
\if (am64x)
    - ICSSG peripheral configuration - Similarly, ICSSG has its own dedicated
      configuration structure called \ref Icssg_Cfg where the application passes
      ICSSG specific configuration parameters, such as:

       - Configuration of the MDIO submodule: \ref Mdio_Cfg.
       - Configuration of TimeSync: \ref IcssgTimeSync_Cfg.
       - Configuration of DMA Resource Partition: \ref EnetRm_ResCfg.
       - Configuration of host port's default: \ref EnetPort_VlanCfg.  This is applicable
         only to ICSSG Switch peripherals which start by default in VLAN aware mode.
\endif

-# Once done with the configuration of the parameters, the UDMA driver has to be
   opened.  Enet utils library provides a helper function called EnetAppUtils_udmaOpen()
   to open the UDMA driver, and returns a handle that can be passed to the
   peripheral configuration parameters, i.e. \ref Cpsw_Cfg::dmaCfg.

-# \ref Enet_open() to open a peripheral, passing the configuration parameters
   previously initialized. \ref Enet_open() function takes the following
   arguments:

    - Peripheral type - This specifies the type or class of the peripheral,
      i.e. CPSW, ICSSG or other.
    - Instance number - This argument specifies the instance number of the
      peripheral.   Refer to @ref enetlld_enetpers section for further information
      about specific peripheral types and instance numbers.
    - Configuration structure - A pointer to the peripheral-specific
      configuration structure, i.e. \ref Cpsw_Cfg or \ref Icssg_Cfg.
    - Size of the configuration structure.

-# If the module is opened successfully, the API will return a valid handle pointer
   to the Enet driver.  It's worth noting that there will be a unique Enet handle
   (\ref Enet_Handle) for each peripheral opened.

-# Attach the core with the Resource Manager (RM) using \ref ENET_PER_IOCTL_ATTACH_CORE.
   IOCTL. To use IOCTLs, the application must have the following:

    - Valid handle to the driver.
    - Core ID, which can be obtained using \ref EnetSoc_getCoreId().
    - Valid IOCTL command.
    - Valid pointer to the IOCTL parameters.

-# Once the application attaches the core with Resource Manager (RM), the IOCTL call
   will return core key which has to be used in all further RM-related calls.

-# A MAC address for the host port is to be obtained using Enet utils helper function
   EnetAppUtils_allocMac() and the corresponding entry in the ALE table can
   be added using EnetAppUtils_addHostPortEntry().

-# Allocate memory for Ring Accelerators, Ethernet packets, etc. Enet LLD examples
   use `EnetAppMemUtils` to take care of all memory allocation and freeing operations.
   The developer can take this as reference or can implement their own memory
   allocation functions.
\if (am64x)
-# ICSSG Switch runs in VLAN aware mode, so it requires the following IOCTLs to setup
   the VLAN:

    - Add a VLAN entry for all ports (host port and the two MAC ports) using
      \ref ICSSG_PER_IOCTL_VLAN_SET_ENTRY.
    - Add a FDB entry for the VLAN Id using \ref ICSSG_FDB_IOCTL_ADD_ENTRY.

   Refer to Enet Multiport example for a reference implementation.
\endif


## Port Open Sequence {#enet_openport_sequence}

The MAC ports can be opened in MAC-to-PHY or MAC-to-MAC mode.  In MAC-to-PHY mode, Enet
LLD's PHY driver state machine will be used to configure the Ethernet PHY.
In MAC-to-MAC mode, the PHY driver will be bypassed entirely.

The link speed and duplexity in MAC-to-PHY can be fixed or auto-negotiated, while in
MAC-to-MAC, both link speed and duplexity can be fixed only and must be provided by
application.

The steps to open and configure ports in either MAC-to-PHY or MAC-to-MAC modes are
shown below.

### MAC-PHY link

-# Set the port number in \ref EnetPer_PortLinkCfg structure.
-# Set the MAC port interface (RMII, RGMII, SGMII, etc) through the layer, sublayer and
   variant fields of \ref EnetPer_PortLinkCfg.mii.
-# The MAC port configuration parameters is peripheral dependent.
    - CPSW MAC Port configuration:
       - Use \ref CpswMacPort_initCfg() to initialize the CPSW MAC port configuration
         parameters to their default values.  Overwrite any parameters as needed.
       - If loopback is to be enabled, set enableLoopback flag in MAC configuration to
         true.
         For loopback to be functional the secure flag in host port ALE entry must be
         set to false.
         The host port entry update can be done using the \ref CPSW_ALE_IOCTL_ADD_UCAST
         command.
\if (am64x)
    - ICSSG MAC Port configuration:
       - Use \ref IcssgMacPort_initCfg() to fill MAC port configuration parameters with
         default values provided by the ICSSG driver.
         The application can then overwrite any configuration parameter as needed, i.e.
         unicast flooding, multicast flooding, etc.
       - ICSSG Switch runs in VLAN aware mode, so the MAC port's default VLAN
         (\ref EnetPort_VlanCfg) must be passed via \ref IcssgMacPort_Cfg::vlanCfg.
       - ICSSG Dual-MAC runs in VLAN unaware mode, so no default VLAN configuration is
         required.
\endif
-# Set PHY configuration parameters: generic and model specific.
    -# Use \ref EnetPhy_initCfg() to initialize the PHY generic parameters to their
       default values, such as auto-negotiation capabilities, strap enable/disable,
       loopback, etc.
    -# Use PHY specific init config functions to initialize model specific parameters.
       This init config function is provided by the Ethernet PHYs supported in Enet LLD.
       For example, Dp83867_initCfg() is used to initialize config params for
       DP83867 PHYs.
    -# Application can also use the Enet helper function EnetBoard_getPhyCfg() to get
       PHY configuration information for PHYs in TI EVMs.
-# Set the link speed and duplexity configuration in \ref EnetPer_PortLinkCfg.linkCfg.
    - For auto-negotiation, use \ref ENET_SPEED_AUTO or \ref ENET_DUPLEX_AUTO.
    - For fixed (manual) configuration, use the fixed speeds and duplexity values in
      defined in \ref Enet_Speed and \ref Enet_Duplexity enumerations.
-# Once all the MAC and PHY configurations are done, the ports can be opened by calling
   the \ref ENET_PER_IOCTL_OPEN_PORT_LINK command.
-# *CPSW Note*: To enable RMII on CPSW9G, external 50 MHz RMII clock from PHY is used on
   SOC RMII_REF_CLOCK pin.  On GESI board, this clock is connected as resistor R225 is
   not populated.
   To get RMII_50MHZ_CLK, resistor R225 needs to be populated. We need to move R226 to
   R225 on GESI board to get this clock.
-# If all the above steps succeeded without any errors, then the Enet driver, the Ethernet
   peripheral and a MAC port have been opened successfully.
-# After opening the port, the driver will run its PHY state machine to configure the
   PHY and eventually get _link up_ status. Application can query the port link using
   \ref ENET_PER_IOCTL_IS_PORT_LINK_UP.
    - Application can check the PHY alive status (i.e. whether PHY is present on the MDIO
      bus) using the \ref ENET_MDIO_IOCTL_IS_ALIVE command.
-# Once link up is detected, application should do the following steps:
    - CPSW:
        - Set the ALE port state to `Forward` state using \ref CPSW_ALE_IOCTL_SET_PORT_STATE
          IOCTL command.
        - Enable the host port of CPSW by calling the \ref ENET_HOSTPORT_IOCTL_ENABLE command.
\if (am64x)
    - ICSSG:
        - Set the MAC port state to `Forward` state using \ref ICSSG_PER_IOCTL_SET_PORT_STATE
          IOCTL command.
\endif

The following code snippet shows how a MAC-PHY link is opened (step 1-6 above). CPSW MAC
port 1 connected to a DP83867 RGMII PHY has been chosen for this example.

\code{.c}
Enet_IoctlPrms prms;
EnetPer_PortLinkCfg portLinkCfg;
EnetMacPort_LinkCfg *linkCfg = &portLinkCfg.linkCfg;
EnetMacPort_Interface *mii = &portLinkCfg.mii;
CpswMacPort_Cfg cpswMacCfg;
EnetPhy_Cfg *phyCfg = &portLinkCfg.phyCfg;
Dp83867_Cfg dp83867Cfg;

/* Step 1 - MAC port 1 */
portLinkCfg->macPort = ENET_MAC_PORT_1;

/* Step 2 - Set port type to RGMII */
mii->layerType    = ENET_MAC_LAYER_GMII;
mii->sublayerType = ENET_MAC_SUBLAYER_REDUCED;
mii->variantType  = ENET_MAC_VARIANT_FORCED;

/* Step 3 - Initialize MAC port configuration parameters */
CpswMacPort_initCfg(&cpswMacCfg);
portLinkCfg.macCfg = &cpswMacCfg;

/* Step 4a - Set PHY generic configuration parameters */
EnetPhy_initCfg(phyCfg);
phyCfg->phyAddr = 0U;

/* Step 4b - DP83867 PHY specific configuration */
Dp83867_initCfg(&dp83867Cfg);
dp83867Cfg.ledMode[1] = DP83867_LED_LINKED_100BTX;
dp83867Cfg.ledMode[2] = DP83867_LED_RXTXACT;
dp83867Cfg.ledMode[3] = DP83867_LED_LINKED_1000BT;
EnetPhy_setExtendedCfg(phyCfg, &dp83867Cfg, sizeof(dp83867Cfg));

/* Step 5 - Set link speed/duplexity to auto-negotiation */
linkCfg->speed     = ENET_SPEED_AUTO;
linkCfg->duplexity = ENET_DUPLEX_AUTO;

/* Step 6 - Open port link */
ENET_IOCTL_SET_IN_ARGS(&prms, &portLinkCfg);
status = Enet_ioctl(hEnet, coreId, ENET_PER_IOCTL_OPEN_PORT_LINK, &prms);
\endcode

### MAC-to-MAC link

-# Set the port number in \ref EnetPer_PortLinkCfg.macPort.
-# Set the MAC port interface (RMII, RGMII, SGMII, etc) through the layer, sublayer and
   variant fields of \ref EnetPer_PortLinkCfg.mii.
-# Initialize the MAC configuration parameters using \ref CpswMacPort_initCfg() and
   manually overwrite any parameters that differ from defaults.
-# Set PHY address to #ENETPHY_INVALID_PHYADDR to indicate that this is a PHY-less
   connection.
-# Set the link speed and duplexity configuration in \ref EnetPer_PortLinkCfg.linkCfg
   to match those of the partner MAC port.  The speed and duplexity must be fixed
   values, they can't be \ref ENET_SPEED_AUTO or \ref ENET_DUPLEX_AUTO as they are
   used for auto-negotiation which is not relevant in MAC-to-MAC mode.
-# Once all the MAC and PHY configurations are done, the ports can be opened by calling
   the \ref ENET_PER_IOCTL_OPEN_PORT_LINK command.
-# If all the above steps succeeded without any errors, then the Enet driver, the Ethernet
   peripheral and a MAC port have been opened successfully.
-# Once link up is detected, application should do the following steps:
    - CPSW:
        - Set the ALE port state to `Forward` state using \ref CPSW_ALE_IOCTL_SET_PORT_STATE
          IOCTL command.
        - Enable the host port of CPSW by calling the \ref ENET_HOSTPORT_IOCTL_ENABLE command.
\if (am64x)
    - ICSSG:
        - Set the MAC port state to `Forward` state using \ref ICSSG_PER_IOCTL_SET_PORT_STATE
          IOCTL command.
\endif

The following code snippet shows how a MAC-to-MAC link is opened (steps 1-6 above). CPSW MAC
port 1 connected to partner MAC using RGMII interface at 1 Gbps.

\code{.c}
Enet_IoctlPrms prms;
EnetPer_PortLinkCfg portLinkCfg;
EnetMacPort_LinkCfg *linkCfg = &portLinkCfg.linkCfg;
EnetMacPort_Interface *mii = &portLinkCfg.mii;
CpswMacPort_Cfg cpswMacCfg;
EnetPhy_Cfg *phyCfg = &portLinkCfg.phyCfg;

/* Step 1 - MAC port 1 */
portLinkCfg->macPort = ENET_MAC_PORT_1;

/* Step 2 - Set port type to RGMII */
mii->layerType    = ENET_MAC_LAYER_GMII;
mii->sublayerType = ENET_MAC_SUBLAYER_REDUCED;
mii->variantType  = ENET_MAC_VARIANT_FORCED;

/* Step 3 - Initialize MAC port configuration parameters */
CpswMacPort_initCfg(&cpswMacCfg);
portLinkCfg.macCfg = &cpswMacCfg;

/* Step 4 - Indicate that this is a PHY-less MAC-to-MAC connection */
phyCfg->phyAddr = ENETPHY_INVALID_PHYADDR;

/* Step 5 - Link speed/duplexity must be explicit set, cannot be set to auto */
linkCfg->speed     = ENET_SPEED_1GBIT;
linkCfg->duplexity = ENET_DUPLEX_FULL;

/* Step 6 - Open port link */
ENET_IOCTL_SET_IN_ARGS(&prms, &portLinkCfg);
status = Enet_ioctl(hEnet, coreId, ENET_PER_IOCTL_OPEN_PORT_LINK, &prms);
\endcode


## Packet Send/Receive Sequence {#enet_pktrxtx_sequence}

-# The DMA channels to enable data transfer are to be opened. It can be
   done using the below steps:

    - Initialize the Tx Channel and Rx Flow parameters using
      \ref EnetDma_initTxChParams() and \ref EnetDma_initRxChParams(),
      respectively.

    - For Tx Channel set the following parameters:
        - EnetUdma_OpenTxChPrms::chNum: Tx Channel number.
        - EnetUdma_OpenTxChPrms::hUdmaDrv: UDMA driver handle obtained before.
        - EnetUdma_OpenTxChPrms::ringMemAllocFxn: EnetMem_allocRingMem.
          or equivalent user preferred function.
        - EnetUdma_OpenTxChPrms::ringMemFreeFxn: EnetMem_freeRingMem
          or equivalent user preferred function.
        - EnetUdma_OpenTxChPrms::numTxPkts: number of Tx packets used to alloc
          ring elements, and descriptors.
        - EnetUdma_OpenTxChPrms::disableCacheOpsFlag: true/false.
        - EnetUdma_OpenTxChPrms::dmaDescAllocFxn: EnetMem_allocDmaDesc
          or equivalent user preferred function.
        - EnetUdma_OpenTxChPrms::dmaDescFreeFxn: EnetMem_freeDmaDesc
          or equivalent user preferred function.
        - EnetUdma_OpenTxChPrms::cbArg: Argument to be used for the callback
          routines.

    - For Rx Flow set the following parameters:
        - EnetUdma_OpenRxFlowPrms::startIdx: Rx flow start index.
        - EnetUdma_OpenRxFlowPrms::flowIdx: Rx flow number.
        - EnetUdma_OpenRxFlowPrms::hUdmaDrv: UDMA driver handle obtained
          previously.
        - EnetUdma_OpenRxFlowPrms::ringMemAllocFxn: EnetMem_allocRingMem
          or equivalent user preferred function.
        - EnetUdma_OpenRxFlowPrms::ringMemFreeFxn: EnetMem_freeRingMem
          or equivalent user preferred function.
        - EnetUdma_OpenRxFlowPrms::numRxPkts: number of Rx packets used to alloc
          ring elements, and descriptors.
        - EnetUdma_OpenRxFlowPrms::rxFlowMtu: Maximum receive packet length for
          this flow.
        - EnetUdma_OpenRxFlowPrms::disableCacheOpsFlag: true/false.
        - EnetUdma_OpenRxFlowPrms::dmaDescAllocFxn: EnetMem_allocDmaDesc
          or equivalent user preferred function.
        - EnetUdma_OpenRxFlowPrms::dmaDescFreeFxn: EnetMem_freeDmaDesc.
          or equivalent user preferred function.
        - EnetUdma_OpenRxFlowPrms::cbArg: Argument to be used for the callback
          routines.

    - After setting the parameters, open the channel and flow using Enet utils helper
      functions EnetAppUtils_openTxCh() and EnetAppUtils_openRxFlow(), respectively.
\if (am64x)
    - ICSSG Switch requires two UDMA RX flows in two different UDMA RX channels, one
      for each ICSSG slice.  The application should use EnetAppUtils_openRxFlowForChIdx()
      helper function instead.  This helper function takes a channel index, so the
      application must call EnetAppUtils_openRxFlowForChIdx() twice, one time for index 0
      and another for index 1.
\endif

-# Now that the Ethernet peripheral (CPSW or ICSSG) and UDMA are configured, the
   application can start the data transfer.

    - To transmit data from application:
        - Call \ref EnetDma_submitTxPktQ() to submit the packets that are
          ready to be transmitted to Tx Free Queue.
        - Call \ref EnetDma_retrieveTxPktQ() to retrieve back the packets
          that were successfully transmitted from Tx Completion Queue.

    - To receive packets that are queued up in the CPSW or ICSSG hardware which are
      ready to be received by the application:
        - Call \ref EnetDma_retrieveRxPktQ() to retrieve packets from Rx
          Completion Queue that are sent to the application.
        - Call \ref EnetDma_submitRxPktQ() to submit new packets to Rx Free
          Queue, so that newly received packets can be copied.
\if (am64x)
    - ICSSG Switch requires two different UDMA RX flows, hence the application must
      retrieve and submit packets via two different queues.  Conversely, the ICSSG
      Dual-MAC only uses a single RX flow, hence packet submit/retrieve happens on
      a single queue.
\endif

\if (am64x)
## Packet Timestamping Sequence {#enet_pktts_sequence}

-# RX packet timestamping doesn't require any enable sequence. The timestamps
   in nanoseconds will be returned to the application along with the DMA packet
   info data, that is, in the \ref EnetUdma_PktTsInfo::rxPktTs field of
   \ref EnetUdma_PktInfo::tsInfo after retrieving RX packets using
   \ref EnetDma_retrieveRxPktQ().
\if (am64x)
-# Application must register a callback function in advance for TX timestamp event
   (\ref ENET_EVT_TIMESTAMP_TX) via \ref Enet_registerEventCb() function.  The
   callback registration can be done at any time before enabling TX timestamping.

-# TX packet timestamping requires to be enabled per packet via DMA packet info
   \ref EnetUdma_PktInfo::tsInfo structure:
    - Enable host TX timestamp by setting \ref EnetUdma_PktTsInfo::enableHostTxTs
      to true.
    - Pass a sequence id which will be used later to correlate the timestamp with
      the packet it belongs to.  The sequence id is passed in
      \ref EnetUdma_PktTsInfo::txPktSeqId.
    - TX packet domain and message type are relevant only for CPSW peripheral.
      They should be set in \ref EnetUdma_PktTsInfo::txPktMsgType and
      \ref EnetUdma_PktTsInfo::txPktDomain.

-# After the packet is submitted for transmission using \ref EnetDma_submitTxPktQ(),
   the timestamp can be retrieved using \ref Enet_poll() function.

    - Application calls \ref Enet_poll() with event type \ref ENET_EVT_TIMESTAMP_TX
      periodically until the register callback is called.
    - The MAC port where timestamp is being polled is specified via Enet_poll()
      `arg` parameter.
\endif
\if (am64x)
          - In ICSSG Dual-MAC peripherals, `arg` parameter must always be set to
            \ref ENET_MAC_PORT_1.
          - In ICSSG Switch peripherals, `arg` parameter must be set to
            \ref ENET_MAC_PORT_1 or \ref ENET_MAC_PORT_2.
\endif
\if (am64x)
    - The returned timestamp is a value in clock cycles for CPSW peripheral and
      a value in nanoseconds for ICSSG peripheral.
\endif
\if (am64x)
    - The sequence id set to before sending packet for transmission in field
      \ref EnetUdma_PktTsInfo::txPktSeqId will be passed to the registered callback.
      Application can use this value to correlate the packet it comes from.
\endif
\endif

## IOCTL Sequence {#enet_ioctl_sequence}

-# Some common IOCTLs have already been mentioned in this document so far.  But
   the application can run any other IOCTL supported by the peripheral.
   There are two kinds of IOCTLs: synchronous and asynchronous.  Asynchronous
   IOCTLs require an additional completion poll step which is explained in
   @ref enet_async_ioctl section of the @ref enet_ioctl_interface document.


## Port Close Sequence {#enet_closeport_sequence}

-# MAC ports that were opened earlier can be closed using
   \ref ENET_PER_IOCTL_CLOSE_PORT_LINK IOCTL command.  This will close the PHY
   state machine associated with this MAC port, if the port was not open in
   MAC-to-MAC mode.  Application can choose to reopen the port at any time
   in the future by following the steps listed in @ref enet_openport_sequence
   section.


## Peripheral Close Sequence {#enet_close_sequence}

-# Disable the host port using the \ref ENET_HOSTPORT_IOCTL_DISABLE command

-# Close the opened Tx Channel and Rx flow:

    - Close the Rx flow using EnetAppUtils_closeRxFlow().
    - Close the Tx Channel using EnetAppUtils_closeTxCh().

-# Detach the core from Resource Manager using the \ref ENET_PER_IOCTL_DETACH_CORE
   command.

-# Enet driver can now be closed and de-initialized using \ref Enet_close()
   and \ref Enet_deinit().


## Deinit Sequence {#enet_deinit_sequence}

This is one-time deinitialization sequence that must be followed when the application
no longer wants to use the Enet LLD.

-# \ref Enet_deinit() should be called to deinitialize the driver. No further Enet
   LLD APIs should be called from this point.

-# Finally, close the UDMA driver using EnetAppUtils_udmaclose()

[Back To Top](@ref enet_integration_guide_top)


## Peripheral-specific {#enetper_specific_handling}
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

\if (am64x)
### ICSSG Dual-MAC {#enetper_icssg_dmac_specific_handling}

The following sections discuss lower level details on how the ICSSG Dual-MAC
and its buffer pool configuration parameters are passed.  This is relevant for
readers who are looking to replace the default Dual-MAC firmware or have
packet buffer requirements not met with the default buffer pool configuration.

#### Firmware {#enetper_icssg_dmac_fw_specific_handling}

Enet LLD provides default firmware for ICSSG Dual-MAC, which is available at
`<enet>/src/per/firmware/icssg/dualmac`.  This firmware is tightly coupled with
a corresponding firmware initialization done internally in Enet LLD.

The firmware information is populated into the ICSSG internal objects (see \ref Icssg_Obj::fw)
by Enet LLD in the corresponding Enet SoC file, i.e. `<enet>/soc/k3/am64x_am243x/enet_soc.c`
for AM64x/AM243x devices.

There are three firmware blobs that must be loaded, one per core in the ICSSG
*slice*: PRU, RTU and TX PRU.  It's done as follows:

<table>
<tr>
  <th>Peripheral
  <th>PRU
  <th>RTU
  <th>TX PRU
<tr>
  <td>ICSSG0 MAC port 1\n (enetType: \ref ENET_ICSSG_DUALMAC, instId: 0)
  <td>RX_PRU_SLICE0_b00_DMac
  <td>RTU0_SLICE0_b00_DMac
  <td>TX_PRU_SLICE0_b00_DMac
<tr>
  <td>ICSSG0 MAC port 2\n (enetType: \ref ENET_ICSSG_DUALMAC, instId: 1)
  <td>RX_PRU_SLICE1_b00_DMac
  <td>RTU0_SLICE1_b00_DMac
  <td>TX_PRU_SLICE1_b00_DMac
<tr>
  <td>ICSSG1 MAC port 1\n (enetType: \ref ENET_ICSSG_DUALMAC, instId: 2)
  <td>RX_PRU_SLICE0_b00_DMac
  <td>RTU0_SLICE0_b00_DMac
  <td>TX_PRU_SLICE0_b00_DMac
<tr>
  <td>ICSSG1 MAC port 2\n (enetType: \ref ENET_ICSSG_DUALMAC, instId: 3)
  <td>RX_PRU_SLICE1_b00_DMac
  <td>RTU0_SLICE1_b00_DMac
  <td>TX_PRU_SLICE1_b00_DMac
<tr>
  <td>ICSSG2 MAC port 1\n (enetType: \ref ENET_ICSSG_DUALMAC, instId: 4)
  <td>RX_PRU_SLICE0_b00_DMac
  <td>RTU0_SLICE0_b00_DMac
  <td>TX_PRU_SLICE0_b00_DMac
<tr>
  <td>ICSSG2 MAC port 2\n (enetType: \ref ENET_ICSSG_DUALMAC, instId: 5)
  <td>RX_PRU_SLICE1_b00_DMac
  <td>RTU0_SLICE1_b00_DMac
  <td>TX_PRU_SLICE1_b00_DMac
</table>

It's worth noting that from application perspective, the ICSSG firmware blobs to
be loaded and the firmware configuration is completely transparent as it doesn't
require any application intervention.


#### Firmware Memory Pools  {#enetper_icssg_dmac_fwmem_specific_handling}

ICSSG firmware requires internal memory for buffer pools and queues to be specified
at firmware configuration time.  This firmware memory is passed into the ICSSG
internal objects (see \ref Icssg_Obj::fwPoolMem) by Enet LLD in the corresponding
Enet SoC file, i.e. `<enet>/soc/k3/am64x_am243x/enet_soc.c` for AM64x/AM243x devices.

There are certain memory requirements for each buffer type for Dual-MAC as shown below:

<table>
<tr>
  <th>Buffer Type
  <th>Number of pools per slice
  <th>Size per buffer pool (bytes)
  <th>Config parameters
<tr>
  <td>Port buffer pools
  <td>0
  <td>0
  <td>\ref Icssg_FwPoolMem::portBufferPoolMem \n
      \ref Icssg_FwPoolMem::portBufferPoolSize \n
      \ref Icssg_FwPoolMem::portBufferPoolNum \n
<tr>
  <td>Host buffer pools
  <td>#ICSSG_DUALMAC_HOST_BUFFER_POOL_NUM
  <td>Configurable
  <td>\ref Icssg_FwPoolMem::hostBufferPoolMem \n
      \ref Icssg_FwPoolMem::hostBufferPoolSize \n
      \ref Icssg_FwPoolMem::hostBufferPoolNum \n
<tr>
  <td>Host egress queues
  <td>#ICSSG_DUALMAC_HOST_EGRESS_QUEUE_NUM
  <td>Configurable. It must include fixed padding size\n of #ICSSG_HOST_EGRESS_BUFFER_PADDING
  <td>\ref Icssg_FwPoolMem::hostEgressQueueMem \n
      \ref Icssg_FwPoolMem::hostEgressQueueSize \n
      \ref Icssg_FwPoolMem::hostEgressQueueNum \n
<tr>
  <td>Scratch buffer
  <td>1
  <td>#ICSSG_SCRATCH_BUFFER_SIZE
  <td>\ref Icssg_FwPoolMem::scratchBufferMem \n
      \ref Icssg_FwPoolMem::scratchBufferSize \n
</table>

Enet LLD provides a default firmware memory configuration in order to reduce the configuration
burden on applications.  The selection of firmware buffer sizes is a trade-off between the
desired amount of packet buffering, the available memory and ICSSG MAC concurrency.

The buffer sizes provided by default in Enet LLD enable simultaneous use of all ICSSGs MACs
in the SoC.  The table below shows the default buffer sizes per ICSSG MAC.


| Buffer Type        | Number of pools | Buffer pool size | Slices | Total size |
| :----------------- | :-------------: | :--------------: | :----: | :--------: |
| Port buffer pools  | 0               | 0                | 1      | 0          |
| Host buffer pools  | 8               | 8 kB             | 1      | 64 kB      |
| Host egress queues | 2               | 8 kB             | 1      | 16 kB      |
| Scratch buffer     | 1               | 2 kB             | 1      | 2 kB       |

If these firmware memory sizes don't meet the requirements of the application, they must
be changed in the corresponding Enet SoC file, i.e. `<enet>/soc/k3/am64x_am243x/enet_soc.c` for
AM64x/AM243x devices.


### ICSSG Switch {#enetper_icssg_swt_specific_handling}

The following sections discuss lower level details on how the ICSSG Switch
and its buffer pool configuration parameters are passed.  This is relevant for
readers who are looking to replace the default Switch firmware or have
packet buffer requirements not met with the default buffer pool configuration.

#### Firmware {#enetper_icssg_swt_fw_specific_handling}

Enet LLD provides default firmware for ICSSG Switch which is available at
`<enet>/src/per/firmware/icssg/switch`.  This firmware is tightly coupled with
a corresponding firmware initialization done internally in Enet LLD.

The firmware information is populated into the ICSSG internal objects (see \ref Icssg_Obj::fw)
by Enet LLD in the corresponding Enet SoC file, i.e. `<enet>/soc/k3/am64x_am243x/enet_soc.c`
for AM64x/AM243x devices.

There are six firmware blobs that must be supplied by the application, one for
each core of the two ICSSG *slices*: PRU, RTU and TX PRU.

<table>
<tr>
  <th>Peripheral
  <th>PRU
  <th>RTU
  <th>TX PRU
<tr>
  <td>ICSSG0\n (enetType: \ref ENET_ICSSG_SWITCH, instId: 0)
  <td>RX_PRU_SLICE0_b00_Swt\n RX_PRU_SLICE1_b00_Swt
  <td>RTU0_SLICE0_b00_Swt\n RTU0_SLICE1_b00_Swt
  <td>TX_PRU_SLICE0_b00_Swt\n TX_PRU_SLICE1_b00_Swt
<tr>
  <td>ICSSG1\n (enetType: \ref ENET_ICSSG_SWITCH, instId: 1)
  <td>RX_PRU_SLICE0_b00_Swt\n RX_PRU_SLICE1_b00_Swt
  <td>RTU0_SLICE0_b00_Swt\n RTU0_SLICE1_b00_Swt
  <td>TX_PRU_SLICE0_b00_Swt\n TX_PRU_SLICE1_b00_Swt
<tr>
  <td>ICSSG2\n (enetType: \ref ENET_ICSSG_SWITCH, instId: 2)
  <td>RX_PRU_SLICE0_b00_Swt\n RX_PRU_SLICE1_b00_Swt
  <td>RTU0_SLICE0_b00_Swt\n RTU0_SLICE1_b00_Swt
  <td>TX_PRU_SLICE0_b00_Swt\n TX_PRU_SLICE1_b00_Swt
</table>

#### Firmware Memory Pools {#enetper_icssg_swt_fwmem_specific_handling}

ICSSG firmware requires internal memory for buffer pools and queues to be specified
at firmware configuration time.  This firmware memory is passed into the ICSSG
internal objects (see \ref Icssg_Obj::fwPoolMem) by Enet LLD in the corresponding
Enet SoC file, i.e. `<enet>/soc/k3/am64x_am243x/enet_soc.c` for AM64x/AM243x devices.

There are certain memory requirements for each buffer type for Switch as shown below:

<table>
<tr>
  <th>Buffer Type
  <th>Number of pools per slice
  <th>Size per buffer pool (bytes)
  <th>Config parameters
<tr>
  <td>Port buffer pools
  <td>#ICSSG_SWITCH_PORT_BUFFER_POOL_NUM
  <td>Configurable
  <td>\ref Icssg_FwPoolMem::portBufferPoolMem \n
      \ref Icssg_FwPoolMem::portBufferPoolSize \n
      \ref Icssg_FwPoolMem::portBufferPoolNum \n
<tr>
  <td>Host buffer pools
  <td>#ICSSG_SWITCH_HOST_BUFFER_POOL_NUM
  <td>Configurable
  <td>\ref Icssg_FwPoolMem::hostBufferPoolMem \n
      \ref Icssg_FwPoolMem::hostBufferPoolSize \n
      \ref Icssg_FwPoolMem::hostBufferPoolNum \n
<tr>
  <td>Host egress queues
  <td>#ICSSG_SWITCH_HOST_EGRESS_QUEUE_NUM
  <td>Configurable. It must include fixed padding size\n of #ICSSG_HOST_EGRESS_BUFFER_PADDING
  <td>\ref Icssg_FwPoolMem::hostEgressQueueMem \n
      \ref Icssg_FwPoolMem::hostEgressQueueSize \n
      \ref Icssg_FwPoolMem::hostEgressQueueNum \n
<tr>
  <td>Scratch buffer
  <td>1
  <td>#ICSSG_SCRATCH_BUFFER_SIZE
  <td>\ref Icssg_FwPoolMem::scratchBufferMem \n
      \ref Icssg_FwPoolMem::scratchBufferSize \n
</table>

Enet LLD provides a default firmware memory configuration in order to reduce the configuration
burden on applications.  The selection of firmware buffer sizes is a trade-off between the
desired amount of packet buffering, the available memory and ICSSG Switch concurrency.

The buffer sizes provided by default in Enet LLD enable simultaneous use of all ICSSGs
Switches in the SoC.  The table below shows the default buffer sizes per ICSSG MAC.


| Buffer Type        | Number of pools | Buffer pool size | Slices | Total size |
| :----------------- | :-------------: | :--------------: | :----: | :--------: |
| Port buffer pools  | 8               | 6 kB             | 2      | 96 kB      |
| Host buffer pools  | 16              | 6 kB             | 2      | 192 kB     |
| Host egress queues | 2               | 8 kB             | 2      | 32 kB      |
| Scratch buffer     | 1               | 2 kB             | 2      | 4 kB       |

If these firmware memory sizes don't meet the requirements of the application, they must
be changed in the corresponding Enet SoC file, i.e. `<enet>/soc/k3/am64x_am243x/enet_soc.c` for
AM64x/AM243x devices.
\endif

\if (am64x)
# Migrating from EMAC LLD {#emac_lld_migration}
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

## ICSSG Migration {#emac_lld_migration_icssg}

### Peripheral open{#emac_lld_migration_icssg_per_open}

EMAC LLD and Enet LLD differ in the kind of parameters passed at open time.
EMAC LLD's `emac_open()` takes a port number, and Enet LLD's \ref Enet_open() takes a
peripheral type and instance id.

Dual-MAC is implemented as separate Enet peripherals of the same type
(\ref ENET_ICSSG_DUALMAC).  Instance number goes from 0 to 5 for AM64x/AM243x device
as there are a total of 3 ICSSG instances, each having 2 MAC ports.

The following tables show the equivalent parameters to use in \ref Enet_open()
for ICSSG Dual-MAC.

<table>
<tr>
  <th>EMAC LLD\n Port Number
  <th>Enet LLD\n Peripheral
<tr>
  <td>EMAC_ICSSG0_PORT0
  <td>enetType: \ref ENET_ICSSG_DUALMAC instId: 0
<tr>
  <td>EMAC_ICSSG0_PORT1
  <td>enetType: \ref ENET_ICSSG_DUALMAC instId: 1
<tr>
  <td>EMAC_ICSSG1_PORT0
  <td>enetType: \ref ENET_ICSSG_DUALMAC instId: 2
<tr>
  <td>EMAC_ICSSG1_PORT1
  <td>enetType: \ref ENET_ICSSG_DUALMAC instId: 3
<tr>
  <td>EMAC_ICSSG2_PORT0
  <td>enetType: \ref ENET_ICSSG_DUALMAC instId: 4
<tr>
  <td>EMAC_ICSSG2_PORT1
  <td>enetType: \ref ENET_ICSSG_DUALMAC instId: 5
</table>

ICSSG Switch is implemented as separate Enet peripherals of the same type
(\ref ENET_ICSSG_SWITCH).  Instance number goes from 0 to 3 for AM64x/AM243x device
as there are a total of 3 ICSSG instances.

The following tables show the equivalent parameters to use in \ref Enet_open()
for ICSSG Switch.

<table>
<tr>
  <th>EMAC LLD\n Port Number
  <th>Enet LLD\n Peripheral
<tr>
  <td>EMAC_SWITCH_PORT
  <td>enetType: \ref ENET_ICSSG_SWITCH instId: 0
<tr>
  <td>EMAC_ICSSG0_SWITCH_PORT
  <td>enetType: \ref ENET_ICSSG_SWITCH instId: 0
<tr>
  <td>EMAC_ICSSG1_SWITCH_PORT
  <td>enetType: \ref ENET_ICSSG_SWITCH instId: 1
<tr>
  <td>EMAC_ICSSG2_SWITCH_PORT
  <td>enetType: \ref ENET_ICSSG_SWITCH instId: 2
</table>

Enet LLD \ref Enet_open() returns a handle of \ref Enet_Handle type, this handle
is used in all other Enet LLD APIs.  This is an important difference with respect
to EMAC LLD, as port number is still required in other EMAC LLD APIs, i.e.
`emac_ioctl`.


### IOCTLs {#emac_lld_migration_icssg_ioctl}

EMAC LLD's `emac_ioctl()` uses a port number as a means to indicate whether the command
is to be run on a Dual-MAC or a Switch, and also to convey information about whether
the command is MAC port or host port related:

  - Dual-MAC - It's the same port number used at open time in `emac_open()`.  There
    is no additional port number for host port IOCTL commands.
  - Switch - There are 4 port numbers per instance:
     - `EMAC_ICSSGn_SWITCH_PORT` used for global settings.
     - `EMAC_ICSSGn_SWITCH_PORT0` used for host port related IOCTLs.
     - `EMAC_ICSSGn_SWITCH_PORT1` used for MAC port 1 related IOCTLs.
     - `EMAC_ICSSGn_SWITCH_PORT2` used for MAC port 2 related IOCTLs.

Note that in the Switch case, the last three port numbers are not used at open time.

In Enet LLD, \ref Enet_ioctl() doesn't take the port number in all cases, but only for
MAC port related IOCTL commands.

IOCTLs that are MAC port related will take the MAC port number (\ref Enet_MacPort)
as IOCTL input args. For example, \ref ENET_MACPORT_IOCTL_SET_PRI_REGEN_MAP command
is MAC port related and it takes the MAC port number in its input arguments
\ref EnetMacPort_SetPriorityRegenMapInArgs::macPort.

IOCTLs that are host port related will not take a port number.  For example,
\ref ICSSG_PER_IOCTL_VLAN_SET_HOSTPORT_DFLT_VID.


#### Asynchronous IOCTLs {#emac_lld_migration_icssg_async_ioctl}

In EMAC LLD, asynchronous IOCTLs return `EMAC_DRV_RESULT_IOCTL_IN_PROGRESS` in `emac_ioctl()`
call which is used to indicate that the application must poll for the operation completion.
Polling is done via `emac_poll_ctrl()` function.

Enet LLD implements asynchronous IOCTLs using a similar mechanism.  First, the application
must register an IOCTL completion callback funcion via \ref Enet_registerEventCb(). When
application has called an asynchronous IOCTL, Enet LLD will return \ref ENET_SINPROGRESS
and the application must call \ref Enet_poll() with event type \ref ENET_EVT_ASYNC_CMD_RESP
until the registered callback is called.

Refer to section \ref enet_async_ioctl of the \ref enet_ioctl_interface document for further
details.


#### IOCTL Command Mappings {#emac_lld_migration_icssg_ioctl_mappings}

The following table shows the mapping between EMAC LLD IOCTL commands/subcommands and Enet
LLD IOCTLs.

<table>
<tr>
  <th>EMAC LLD\n IOCTL cmd
  <th>EMAC LLD\n IOCTL subcmd
  <th>Enet LLD\n IOCTL cmd
<tr>
  <td>`EMAC_IOCTL_PROMISCOUS_MODE_CTRL`
  <td>
  <td>\ref ICSSG_MACPORT_IOCTL_ENABLE_PROMISC_MODE \n \ref ICSSG_MACPORT_IOCTL_DISABLE_PROMISC_MODE
<tr>
  <td rowspan="2">`EMAC_IOCTL_UC_FLOODING_CTRL`
  <td>`EMAC_IOCTL_PORT_UC_FLOODING_ENABLE`
  <td>\ref ICSSG_MACPORT_IOCTL_ENABLE_UCAST_FLOOD
<tr>
  <td>`EMAC_IOCTL_PORT_UC_FLOODING_DISABLE`
  <td>\ref ICSSG_MACPORT_IOCTL_DISABLE_UCAST_FLOOD
<tr>
  <td rowspan="2">`EMAC_IOCTL_MC_FLOODING_CTRL`
  <td>`EMAC_IOCTL_PORT_MC_FLOODING_ENABLE`
  <td>\ref ICSSG_MACPORT_IOCTL_ENABLE_MCAST_FLOOD
<tr>
  <td>`EMAC_IOCTL_PORT_MC_FLOODING_DISABLE`
  <td>\ref ICSSG_MACPORT_IOCTL_DISABLE_MCAST_FLOOD
<tr>
  <td rowspan="4">`EMAC_IOCTL_PORT_STATE_CTRL`
  <td>`EMAC_IOCTL_PORT_STATE_DISABLE`
  <td>\ref ICSSG_PER_IOCTL_SET_PORT_STATE
<tr>
  <td>`EMAC_IOCTL_PORT_STATE_BLOCKING`
  <td>\ref ICSSG_PER_IOCTL_SET_PORT_STATE
<tr>
  <td>`EMAC_IOCTL_PORT_STATE_FORWARD`
  <td>\ref ICSSG_PER_IOCTL_SET_PORT_STATE
<tr>
  <td>`EMAC_IOCTL_PORT_STATE_FORWARD_WO_LEARNING`
  <td>\ref ICSSG_PER_IOCTL_SET_PORT_STATE
<tr>
  <td rowspan="3">`EMAC_IOCTL_ACCEPTABLE_FRAME_CHECK_CTRL`
  <td>`EMAC_IOCTL_ACCEPTABLE_FRAME_CHECK_ONLY_VLAN_TAGGED`
  <td>\ref ICSSG_MACPORT_IOCTL_SET_ACCEPT_FRAME_CHECK
<tr>
  <td>`EMAC_IOCTL_ACCEPTABLE_FRAME_CHECK_ONLY_UN_TAGGED_PRIO_TAGGED`
  <td>\ref ICSSG_MACPORT_IOCTL_SET_ACCEPT_FRAME_CHECK
<tr>
  <td>`EMAC_IOCTL_ACCEPTABLE_FRAME_CHECK_ALL_FRAMES`
  <td>\ref ICSSG_MACPORT_IOCTL_SET_ACCEPT_FRAME_CHECK
<tr>
  <td rowspan="4">`EMAC_IOCTL_VLAN_CTRL`
  <td>`EMAC_IOCTL_VLAN_SET_DEFAULT_TBL`
  <td>\ref ICSSG_PER_IOCTL_VLAN_RESET_TABLE
<tr>
  <td>`EMAC_IOCTL_VLAN_SET_ENTRY`
  <td>\ref ICSSG_PER_IOCTL_VLAN_SET_ENTRY
<tr>
  <td>`EMAC_IOCTL_VLAN_SET_DEFAULT_VLAN_ID`
  <td>\ref ICSSG_PER_IOCTL_VLAN_SET_HOSTPORT_DFLT_VID \n \ref ICSSG_PER_IOCTL_VLAN_SET_MACPORT_DFLT_VID
<tr>
  <td>`EMAC_IOCTL_VLAN_GET_ENTRY`
  <td>\ref ICSSG_PER_IOCTL_VLAN_GET_ENTRY
<tr>
  <td>`EMAC_IOCTL_PORT_PRIO_MAPPING_CTRL`
  <td>
  <td>\ref ENET_MACPORT_IOCTL_SET_EGRESS_QOS_PRI_MAP
<tr>
  <td>`EMAC_IOCTL_PRIO_REGEN_CTRL`
  <td>
  <td>\ref ENET_MACPORT_IOCTL_SET_PRI_REGEN_MAP
<tr>
  <td>`EMAC_IOCTL_SPECIAL_FRAME_PRIO_CONFIG`
  <td>
  <td>\ref IcssgMacPort_Cfg::specialFramePrio
<tr>
  <td>`EMAC_IOCTL_FDB_AGEING_TIMEOUT_CTRL`
  <td>
  <td>\ref ICSSG_FDB_IOCTL_SET_AGING_PERIOD
<tr>
  <td rowspan="4">`EMAC_IOCTL_FDB_ENTRY_CTRL`
  <td>`EMAC_IOCTL_FDB_ENTRY_ADD`
  <td>\ref ICSSG_FDB_IOCTL_ADD_ENTRY
<tr>
  <td>`EMAC_IOCTL_FDB_ENTRY_DELETE`
  <td>\ref ICSSG_FDB_IOCTL_REMOVE_ENTRY
<tr>
  <td>`EMAC_IOCTL_FDB_ENTRY_DELETE_ALL`
  <td>\ref ICSSG_FDB_IOCTL_REMOVE_ALL_ENTRIES
<tr>
  <td>`EMAC_IOCTL_FDB_ENTRY_DELETE_ALL_AGEABLE`
  <td>\ref ICSSG_FDB_IOCTL_REMOVE_AGEABLE_ENTRIES
<tr>
  <td>`EMAC_IOCTL_INTERFACE_MAC_CONFIG`
  <td>`EMAC_IOCTL_INTERFACE_MAC_ADD`
  <td>\ref ICSSG_HOSTPORT_IOCTL_SET_MACADDR \n \ref ICSSG_MACPORT_IOCTL_SET_MACADDR
<tr>
  <td>`EMAC_IOCTL_INGRESS_RATE_LIMIT_CTRL`
  <td>
  <td>\ref ICSSG_MACPORT_IOCTL_SET_INGRESS_RATE_LIM
<tr>
  <td rowspan="9">`EMAC_IOCTL_FRAME_PREEMPTION_CTRL`
  <td>`EMAC_IOCTL_PREEMPT_TX_ENABLE`
  <td>\ref ICSSG_MACPORT_IOCTL_PREEMPT_TX_ENABLE
<tr>
  <td>`EMAC_IOCTL_PREEMPT_TX_DISABLE`
  <td>\ref ICSSG_MACPORT_IOCTL_PREEMPT_TX_DISABLE
<tr>
  <td>`EMAC_IOCTL_PREEMPT_GET_TX_ENABLE_STATUS`
  <td>\ref ICSSG_MACPORT_IOCTL_PREEMPT_GET_TX_ENABLE_STATUS
<tr>
  <td>`EMAC_IOCTL_PREEMPT_GET_TX_ACTIVE_STATUS`
  <td>\ref ICSSG_MACPORT_IOCTL_PREEMPT_GET_TX_ACTIVE_STATUS
<tr>
  <td>`EMAC_IOCTL_PREEMPT_VERIFY_ENABLE`
  <td>\ref ICSSG_MACPORT_IOCTL_PREEMPT_VERIFY_ENABLE
<tr>
  <td>`EMAC_IOCTL_PREEMPT_VERIFY_DISABLE`
  <td>\ref ICSSG_MACPORT_IOCTL_PREEMPT_VERIFY_DISABLE
<tr>
  <td>`EMAC_IOCTL_PREEMPT_GET_VERIFY_STATE`
  <td>\ref ICSSG_MACPORT_IOCTL_PREEMPT_GET_VERIFY_STATE
<tr>
  <td>`EMAC_IOCTL_PREEMPT_GET_MIN_FRAG_SIZE_LOCAL`
  <td>\ref ICSSG_MACPORT_IOCTL_PREEMPT_GET_MIN_FRAG_SIZE_LOCAL
<tr>
  <td>`EMAC_IOCTL_PREEMPT_SET_MIN_FRAG_SIZE_REMOTE`
  <td>\ref ICSSG_MACPORT_IOCTL_PREEMPT_SET_MIN_FRAG_SIZE_REMOTE
  <td>
<tr>
  <td>`EMAC_IOCTL_CUT_THROUGH_PREEMPT_SELECT`
  <td>
  <td>\ref IcssgMacPort_Cfg::queuePreemptMode \n \ref IcssgMacPort_Cfg::queueForwardMode
<tr>
  <td rowspan="6">`EMAC_IOCTL_SPEED_DUPLEXITY_CTRL`
  <td>`EMAC_IOCTL_SPEED_DUPLEXITY_10HD`
  <td rowspan="6">No longer available as explicit IOCTLs.\n
      Speed and duplexity are passed as arguments to \ref ENET_PER_IOCTL_OPEN_PORT_LINK in either
      auto-negotiation or manual modes.
<tr>
  <td>`EMAC_IOCTL_SPEED_DUPLEXITY_10FD`
<tr>
  <td>`EMAC_IOCTL_SPEED_DUPLEXITY_100HD`
<tr>
  <td>`EMAC_IOCTL_SPEED_DUPLEXITY_100FD`
<tr>
  <td>`EMAC_IOCTL_SPEED_DUPLEXITY_GIGABIT`
<tr>
  <td>`EMAC_IOCTL_SPEED_DUPLEXITY_DISABLE`
</table>


### Packet send and receive {#emac_lld_migration_icssg_dma}

EMAC LLD relies on `emac_send()` and `emac_poll_pkt()` for packet transmission and reception,
respectively.

- For transmission, application populates a descriptor with packet information and passes it
  to `emac_send()`.  The descriptor provides fields for packet buffer, packet length, traffic
  class, etc.

- For reception, application registers a callback (`rx_pkt_cb`) at open time, which will be
  called by the driver when application calls `emac_poll_pkt()` and packets have been received.

Internally, EMAC LLD would open all required TX channels and RX flows.

Enet LLD takes a different approach, provides a queue oriented mechanism for packet
submission and retrieval.

<table>
<tr>
  <th>Enet LLD API
  <th>Description
<tr>
  <td>\ref EnetDma_retrieveRxPktQ()
  <td>Called by application to retrieve *ready* packets from the driver, that is, new
      full packets.
<tr>
  <td>\ref EnetDma_submitRxPktQ()
  <td>Callen by application to return *free* packets that the application has consumed
      and is ready to recycle.
<tr>
  <td>\ref EnetDma_submitTxPktQ()
  <td>Called by application to submit *ready* packets to the driver for transmission.
<tr>
  <td>\ref EnetDma_retrieveTxPktQ()
  <td>Called by application to retrieve *done* packets which the driver has already
      used for transmission and are ready to be reused for future transmission.
</table>

The queues passed to above APIs are created by the application using EnetQueue APIs, such as
\ref EnetQueue_initQ(), \ref EnetQueue_enq() and \ref EnetQueue_deq().

The queues are composed of \ref EnetDma_Pkt objects, which is the abstraction provided
by Enet LLD for a packet.  The native type of the packet object in AM64x/AM243x/J721E devices
is \ref EnetUdma_PktInfo.  Note that this same packet object type is used for transmission
and reception.

\endif

[Back To Top](@ref enet_integration_guide_top)
