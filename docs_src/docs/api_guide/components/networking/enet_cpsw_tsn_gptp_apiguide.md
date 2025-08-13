# Ethernet TSN and gPTP Stack - API and Integration Guide {#ENET_CPSW_TSN_GPTP}

[TOC]

# Pre-requisites
Reader are expected to have basic knowledge on below IEEE specifications
- Standard ethernet (IEEE 802.1)
- Timing and Synchronization for Time-Sensitive Applications - gPTP (IEEE 802.1AS-2020)
- Forwarding and Queuing Enhancements for Time-Sensitive Streams (IEEE 802.1Qav)
- Enhancements for Scheduled Traffic (IEEE 802.1Qbv)
- Frame Preemption (IEEE 802.1Qbu)
- YANG Data Modeling Language (rfc7950)


# Introduction
This guide is intended to enhance user's understanding of the TSN stack and provide guidance on how to seamlessly integrate TSN modules into their own applications.

# Demo and Examples
\ref EXAMPLES_ENET_CPSW_TSN_GPTP_BRIDGE

\ref EXAMPLES_ENET_CPSW_TSN_GPTP_TR

\ref EXAMPLES_ENET_CPSW_TSN_GPTP_TT

# TSN Stack

## Compilation
TSN stack library is pre-built in SDK and located in ``<mcu_plus_sdk>/source/networking/enet/core/examples/tsn`` directory as .lib files. Any changes in the stack source code needs rebuild of library.
In order to re-build the library, follow the below steps-
\code
cd  <mcu_plus_sdk>
gmake DEVICE:=@VAR_SOC_NAME PROFILE:=debug libs-clean
gmake DEVICE:=@VAR_SOC_NAME PROFILE:=release libs-clean
gmake DEVICE:=@VAR_SOC_NAME PROFILE:=debug libs
gmake DEVICE:=@VAR_SOC_NAME PROFILE:=release libs
\endcode

## Modules

The TSN Stack library is composed of the following source modules:

 Module Name  | lcoation | Description
 -------------|-----------|-----------
 Unibase      | `<mcu_plus_sdk>/source/networking/tsn/tsn_unibase` | Universal utility libraries that are platform-independent
 Combase      | `<mcu_plus_sdk>/source/networking/tsn/tsn_combase` | Communication utility libraries that provide support for functions like sockets, mutexes, and semaphores
 Uniconf      | `<mcu_plus_sdk>/source/networking/tsn/tsn_uniconf` | Universal configuration daemon for Yang, provides APIs for developing a client application which retreives/writes yang parameters from/to database
 gPTP         | `<mcu_plus_sdk>/source/networking/tsn/tsn_gptp`    | Implementation of the IEEE 802.1 AS gptp protocol

## Stack Initialization {#ENET_CPSW_TSN_STACK_INITIALIZATION}

A reference example of the TSN stack initialization can be found in ``<mcu_plus_sdk>/source/networking/enet/core/examples/tsn/tsninit.c``.
Prior to any module calls, it is necessary to initialize the unibase library once.
This can be achieved by invoking the ``EnetApp_initTsnByCfg()`` function.

## Logging {#ENET_CPSW_TSN_LOGGING}

By default, TSN logs are directed to STDOUT using the fwrite function:
``fwrite(str, 1, strlen(str), stdout);``
However, it is possible to customize the log output by setting the ``Logger_init(cfg->consoleOutCb)``
callback during the initialization of the unibase library in the ``EnetApp_initTsnByCfg()``
function.

When a callback function is assigned to ``consoleOutCb`` it takes a significant amount
of time to log to the output device, it is advisable to log to a buffer and utilize
another task to log from the buffer to the desired output device.  This approach
helps to prevent log delays that could adversely affect the gPTP task, such as
incorrect *Sync* and *FollowUp* intervals.  The option to log to a buffer is
supported when the ``TSN_USE_LOG_BUFFER`` macro is defined.

To enable specific log levels, you can modify the ub_log_initstr parameter in
the ``EnetApp_initTsnByCfg(``) function. There are eight log levels available::
\code
    UBL_NONE = 0
    UBL_FATAL = 1
    UBL_ERROR = 2
    UBL_WARN = 3
    UBL_INFO = 4
    UBL_INFOV = 5
    UBL_DEBUG = 6
    UBL_DEBUGV = 7
\endcode

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

## Starting uniconf and gPTP {#ENET_CPSW_TSN_UNICONF_INITIALIZATION}

The gPTP functionality operates in a separate task, alongside the universal configuration daemon "uniconf".
To start these tasks, use the ``EnetApp_startTsn()`` in tsninit.c as reference .

This function will start the uniconf and gPTP tasks.

## TSN Deinitialization  {#ENET_CPSW_TSN_STACK_DEINITIALIZATION}

To deinitialize the TSN modules, you can invoke the ``EnetApp_stopTsn();`` and ``EnetApp_deInitTsn();`` functions.

# Integration
## Source integration {#ENET_CPSW_TSN_SOURCE_INTEGRATION}

To integrate the TSN stack into your application, follow these steps:

- Initialize Enet LLD and setup board dependencies.  In the TSN example application,
  the initialization routines can be found at ``<mcu_plus_sdk>/source/networking/enet/core/examples/tsn/tsnapp_cpsw_main.c.c``,
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
  ``EnetApp_initTsnByCfg()``.

- Initialize the *combase* Ethernet device table as the following code snippet::

\code
      for (i = 0; i < gEnetAppCfg.numMacPort; i++)
      {
          snprintf(&g_netdevices[i][0], CB_MAX_NETDEVNAME, "tilld%d", i);
          ethdevs[i].netdev = g_netdevices[i];
          ethdevs[i].macport = gEnetAppCfg.macPorts[i];
          memcpy(ethdevs[i].srcmac, gEnetAppCfg.macAddr, ENET_MAC_ADDR_LEN);
      }

      cb_lld_init_devs_table(ethdevs, i, enetType, instId);
\endcode

  In the above code, the Ethernet device name and MAC port are required to create
  a virtual Ethernet device table that facilitates the conversion between the device
  name and the MAC port number.

In the above code, the Ethernet device name and MAC port are required to create
  a virtual Ethernet device table that facilitates the conversion between the device
  name and the MAC port number.

  Please ensure that the ``ethdevs[i].netdev`` pointer points to a global memory
  location rather than a stack memory. The *combase* layer will reference this
  address instead of making a copy.

  If the "ethdevs[i].srcmac" is either unset or set to all zeroes, the "combase" will
  allocate a source MAC address. If the user's application requires manual assignment
  of a source MAC address to the interface, please ensure to set the "ethdevs[i].srcmac" variable accordingly.

- To provide syscfg values to the TSN stack, you can use the ``cb_socket_set_lldcfg_update_cb()```
  The parameter for this function is a callback function that updates the cb_socket_lldcfg_update_t values.

- Start the gPTP task by calling ``EnetApp_startTsn()``. This will start the necessary threads for gPTP and uniconf.
  The uniconf, gPTP tasks are stored in ``EnetApp_ModuleCtx_t gModCtxTable`` task's table.

## Uniconf configuration {#ENET_CPSW_TSN_UNICONF_CONFIGURATION}

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

## gPTP Yang Config Parameters {#ENET_CPSW_TSN_YANG_CONFIG_PARAMS}

This section describes the standard Yang parameters utilized for gPTP.

This implementation assumes user familiarity with the IEEE 1588 and 802.1AS YANG standards, and the mapping of these standards to our key-based enumeration. The gptp stack is developed based on below YANG standard:

- ieee1588: https://github.com/YangModels/yang/blob/main/standard/ieee/published/1588/ieee1588-ptp-tt.yang
- 802.1AS: https://github.com/YangModels/yang/blob/main/standard/ieee/published/802.1/ieee802-dot1as-gptp.yang

### gPTP string-based to key-based mapping
First of all, the key-based (uint8_t) enums which are used to map with YANG (string-based) are generated during initialization.
For example, the corresponding ptp generated enum are defined in ``<mcu_plus_sdk>/source/networking/tsn/tsn-stack/tsn_uniconf/yangs/cores/generated/ieee1588-ptp-tt.h``

This section will explain on how to map between generated key-based and string-based.

Consider priority1 (assuming 248) in ieee1588-ptp-tt.yang, which should have string-based like below:

``/ieee1588-ptp-tt/ptp/instances/instance|instance-index:0|/default-ds/priority1``

The corresponding key-based array for priority1 will be:
```
k[0]=IEEE1588_PTP_TT_RW
k[1]=IEEE1588_PTP_TT_PTP
k[2]=IEEE1588_PTP_TT_INSTANCES
k[3]=IEEE1588_PTP_TT_INSTANCE
k[4]=IEEE1588_PTP_TT_DEFAULT_DS
k[5]=IEEE1588_PTP_TT_PRIORITY1
```

- RW/RO entry: The `k[0]` is IEEE1588_PTP_TT_RW to indicate the entry is configurable, similar with YANG model ``config true;``.

User must take note and understand the YANG standard, to understand the entry is configurable or not.

In our example, the setting 'true' in ``EnetApp_DbKeyVal_IntItem_t`` is to indicate whether the entry is RW or RO.

- The list's key ``|instance-index:0|``: will be described in next section, together with ``ieee1588-ptp-tt_access.*``

### ieee1588-ptp-tt_access.c/h
Beside of generated files eg:ieee1588-ptp-tt.h, there is APIs helper for each YANG generated file, named *access.h/c. Eg: for ieee1588-ptp-tt.yang, the corresponding helper will be ieee1588-ptp-tt_access.c/h.

The ``ieee1588-ptp-tt_access`` provide wrapper APIs for quicker access ``ieee1588-ptp-tt`` entries. Consider below set of APIs:

```
int ydbi_get_item_ptk3vk0(yang_db_item_access_t *ydbia, void **rval, uint32_t instIndex,
			  uint8_t k1, uint8_t k2, uint8_t k3, bool status);

int ydbi_rel_item_ptk3vk0(yang_db_item_access_t *ydbia, uint8_t instIndex,
			  uint8_t k1, uint8_t k2, uint8_t k3, bool status);

int ydbi_set_item_ptk3vk0(yang_db_item_access_t *ydbia, uint8_t instIndex,
			  uint8_t k1, uint8_t k2, uint8_t k3,
			  bool status, void *value, uint32_t vsize, uint8_t notice);
int ydbi_del_item_ptk3vk0(yang_db_item_access_t *ydbia, uint8_t instIndex,
			  uint8_t k1, uint8_t k2, uint8_t k3,
			  bool status);
```

The suffix of `ptk3vk0` can be extracted into 3 parts:
- `pt` is for ieee1588-ptp-tt
- `k3` means the input will have 3 keys and
- `vk0` no value key after `instance-index`

The `uint32_t instIndex` is used to get corresponding `|instance-index:0|` which we need in previous section. Please refer to ``set_dpara_k4vk1`` for more detail of using instIndex.

The higher level DB API wrapper, which is using specificly for gptp, is defined in ``<mcu_plus_sdk>/source/networking/tsn/tsn-stack/tsn_gptp/gptpconf/gptpgcfg.h``.


### Default example configurations
Let's walk-through all of yang configuration those are using in ``<mcu_plus_sdk>/source/networking/enet/core/examples/tsn/gptp_init.c``.

| KEY | Refer YANG's items | Short Explain |
|---------:|:--------:|----------|
| IEEE1588_PTP_TT_LOG_ANNOUNCE_INTERVAL    | ieee1588-ptp-tt:log-announce-interval | Logarithm to the base 2 (log2(x)) of Announce Interval     |
| IEEE1588_PTP_TT_ANNOUNCE_RECEIPT_TIMEOUT      |   ieee1588-ptp-tt:announce-receipt-timeout    | same with GPTP Cap timeout, apply for Announce packets   |
| IEEE1588_PTP_TT_INITIAL_LOG_ANNOUNCE_INTERVAL      |   ieee1588-ptp-tt:log-announce-interval    | Default Init log2(x) of Announce Interval    |
| IEEE1588_PTP_TT_INITIAL_LOG_SYNC_INTERVAL      |   ieee802-dot1as-gptp:initial-log-sync-interval    | Default Init log2(x) of Sync Interval    |
| IEEE1588_PTP_TT_LOG_SYNC_INTERVAL      |   ieee1588-ptp-tt:log-sync-interval    | log2(x) of Sync Interval    |
| IEEE1588_PTP_TT_SYNC_RECEIPT_TIMEOUT      |   ieee802-dot1as-gptp:sync-receipt-timeout    | same with GPTP Cap timeout, apply for Sync packets    |
| IEEE1588_PTP_TT_CURRENT_LOG_SYNC_INTERVAL      |   ieee802-dot1as-gptp:current-log-sync-interval    | Current value of log-sync-interval      |
| IEEE1588_PTP_TT_INITIAL_LOG_PDELAY_REQ_INTERVAL      |   ieee802-dot1as-gptp:initial-log-pdelay-req-interval    | Default Init log2(x) of PdelayReq Interval    |
| IEEE1588_PTP_TT_CURRENT_LOG_PDELAY_REQ_INTERVAL      |   ieee802-dot1as-gptp:current-log-pdelay-req-interval    | Current value of log-pdelay-req-interval    |
| IEEE1588_PTP_TT_INITIAL_LOG_GPTP_CAP_INTERVAL      |   ieee802-dot1as-gptp:initial-log-gptp-cap-interval    |  Default Init log2(x) of GptpCapable Interval    |
| IEEE1588_PTP_TT_USE_MGT_LOG_GPTP_CAP_INTERVAL      |   ieee802-dot1as-gptp:use-mgt-log-gptp-cap-interval    | False: Allow gptp-cap-interval can be changed by message Interval Request Signaling and True: Not allow, use default mgt-log-gptp-cap-interval  |
| IEEE1588_PTP_TT_MGT_LOG_GPTP_CAP_INTERVAL      |   ieee802-dot1as-gptp:mgt-log-gptp-cap-interval    | Default value in case of use-mgt-log-gptp-cap-interval is False    |
| IEEE1588_PTP_TT_GPTP_CAP_RECEIPT_TIMEOUT      |   ieee802-dot1as-gptp:gptp-cap-receipt-timeout    | GPTP Capable timeout time =  Cap Interval * gptp-cap-receipt-timeout   |
| IEEE1588_PTP_TT_CURRENT_LOG_GPTP_CAP_INTERVAL      |   ieee802-dot1as-gptp:current-log-gptp-cap-interval    | Current gptp-cap-interval value   |
| IEEE1588_PTP_TT_ALLOWED_LOST_RESPONSES      |   ieee802-dot1as-gptp:allowed-lost-responses    | Maximum number of lost Pdelay_Respond/Follow_up of a PdelayRequest    |
| IEEE1588_PTP_TT_ALLOWED_FAULTS      |   ieee802-dot1as-gptp:allowed-faults    | Number of faults above which asCapable is set to false    |
| IEEE1588_PTP_TT_MEAN_LINK_DELAY_THRESH      |   ieee802-dot1as-gptp:mean-link-delay-thresh    | Threshold of a pdelay calculated. If it's higher than this threshold, port is not consider capable    |
| IEEE1588_PTP_TT_PORT_ENABLE      |   ieee1588-ptp-tt:port-enable    | Must be True    |
| IEEE1588_PTP_TT_MINOR_VERSION_NUMBER      |   ieee1588-ptp-tt:minor-version-number    | Minor PTP version number    |
| IEEE1588_PTP_TT_INITIAL_ONE_STEP_TX_OPER      |   ieee802-dot1as-gptp:initial-one-step-tx-oper    | Init value of using one-step-tx, in our stack, we ONLY apply two-step-tx, so the value must be False   |
| IEEE1588_PTP_TT_CURRENT_ONE_STEP_TX_OPER      |   ieee802-dot1as-gptp:current-one-step-tx-oper    | Current value of one-step-tx    |
| IEEE1588_PTP_TT_MGT_ONE_STEP_TX_OPER      |   ieee802-dot1as-gptp:mgt-one-step-tx-oper    | Default case one-step-tx if use-mgt-one-step-tx-oper  is true      |
| IEEE1588_PTP_TT_USE_MGT_ONE_STEP_TX_OPER      |   ieee802-dot1as-gptp:use-mgt-one-step-tx-oper    | Must be False as we do not support one-step-tx    |
| IEEE1588_PTP_TT_PRIORITY1      |   ieee1588-ptp-tt:priority1    | Use to configure ptp priority1    |
| IEEE1588_PTP_TT_PRIORITY2      |   ieee1588-ptp-tt:priority2    | Use to configure ptp priority2    |
| IEEE1588_PTP_TT_EXTERNAL_PORT_CONFIG_ENABLE      |   ieee1588-ptp-tt:external-port-config-enable    |  We are not allow external-port-config-enable, so it should be False   |
| IEEE1588_PTP_TT_TIME_SOURCE      |   ieee1588-ptp-tt:time-source    | Config time-source, refer YANG for possible value    |
| IEEE1588_PTP_TT_PTP_TIMESCALE      |   ieee1588-ptp-tt:ptp-timescale    | Config ptp-timescale, refer YANG for possible value    |
| IEEE1588_PTP_TT_CLOCK_CLASS      |   ieee1588-ptp-tt:clock-class    | Config clock-class, refer YANG for possible value    |
| IEEE1588_PTP_TT_CLOCK_ACCURACY      |   ieee1588-ptp-tt:clock-accuracy    | Config clock-accuracy, refer YANG for possible value    |
| IEEE1588_PTP_TT_OFFSET_SCALED_LOG_VARIANCE      |   ieee1588-ptp-tt:offset-scaled-log-variance    | Config offset-scaled-log-variance, refer YANG for possible value    |
| IEEE1588_PTP_TT_INGRESS_LATENCY      |   ieee1588-ptp-tt:ingress-latency    | Config this if you know the internal ingress-latency of a rx ptp packets. Currently the value is 0    |
| IEEE1588_PTP_TT_EGRESS_LATENCY      |   ieee1588-ptp-tt:egress-latency    | Config this if you know the internal egress-latency of a tx ptp packets. Currently the value is 0    |

For more accessible parameters along with the default values in gPTP example, please refer to the file located at:
-``<mcu_plus_sdk>/source/networking/tsn/tsn-stack/tsn_gptp/gptpconf/gptp-yangconfig.xml``.

## gPTP Non-Yang Config Parameters

In addition to the standard Yang parameters, gPTP also includes a set of non-Yang  configuration parameters that are specific to its implementation. 

### gptpgcfg wrapper APIs to modify Non-Yang parameter
To modify the default value of the Non-Yang parameters, please refer to the ``gptp_nonyang_config()`` function
in the ``gptp_init.c`` file. This function will invokes ``gptpgcfg_set_item()`` to configure each params:

\code
int gptpgcfg_set_item(uint8_t gptpInstanceIndex, uint8_t confitem,
                        bool status, void *value, uint32_t vsize);
\endcode

- ``gptpInstanceIndex`` : Index of the gptp to be configured. For TI FreeRTOS there is only one instance is applicable, this should be set to 0.

- ``confitem``: The configuration item. To create an item, prepend the parameter name from the gptp_nonyangconfig.xml
with the "XL4_EXTMOD_XL4GPTP" prefix, as follows: "USE_HW_PHASE_ADJUSTMENT" becomes "XL4_EXTMOD_XL4GPTP_USE_HW_PHASE_ADJUSTMENT."

- ``status``: ``YDBI_CONFIG`` or ``YDBI_STATUS``.  ``YDBI_CONFIG`` is used
to configure the parameter at runtime.  ``YDBI_STATUS`` is used to
configure the parameter at runtime and store the value in the database
for persistence.
- ``value``: Pointer to the value to be set.
- ``vsize``: Size of the value to be set, for the items.

Example usage:
``gptpgcfg_set_item(gpoptd.instnum, XL4_EXTMOD_XL4GPTP_USE_HW_PHASE_ADJUSTMENT,
  YDBI_CONFIG, &use_hwphase, sizeof(use_hwphase));``
- As a reference, please consult the ``<mcu_plus_sdk>/source/networking/enet/core/examples/tsn/gptp_init.c``
file.

- The configuration must be done before calling the ``gptpman_run()`` function
  inside the ``gptp_init.c`` file, and it should follow the the gptpgcfg_init function call.

### Default example configurations

| KEY | Short Explain |
|:---------:|:----------|
|SINGLE_CLOCK_MODE| Set 1 for single clock with multiple ports|
|USE_HW_PHASE_ADJUSTMENT| Set 1 to make the adjustment apply on HW clock|
|FREQ_OFFSET_IIR_ALPHA_START_VALUE| ptp clock offset and freq adjustment IIR filter coefficients reciprocal number is used at the beginning|
|FREQ_OFFSET_IIR_ALPHA_STABLE_VALUE| ptp clock offset and freq adjustment IIR filter coefficients reciprocal number is used after stable time|
|PHASE_OFFSET_IIR_ALPHA_START_VALUE| ptp clock offset and freq adjustment IIR filter coefficients reciprocal number is used at the beginning|
|PHASE_OFFSET_IIR_ALPHA_STABLE_VALUE| ptp clock offset and freq adjustment IIR filter coefficients reciprocal number is used after stable time|
|MAX_DOMAIN_NUMBER| Support up to 2 domains|
|QUICK_SYNC_ALGO| Enables the slave clock to match the master's rate before applying phase offset adjustments.|
|SKIP_FREQADJ_COUNT_MAX| Number of Sync message cycle in which freq adjustment will be skipped after offset has just been adjusted.|
|PHASE_OFFSET_ADJUST_BY_FREQ| Sets the phase offset adjustment threshold in nanoseconds.|
|FREQ_OFFSET_STABLE_PPB| Stable adjustment determination and thresholds freq. adjustment is considered stable if delta of adj rate is less then this threshold|
|FREQ_OFFSET_UPDATE_MRATE_PPB| update freq offset only when the abs of diff to the new rate is bigger than this|
|STATIC_PORT_STATE_SLAVE_PORT| -1: Enable BMCA mode, 0: Disable BMCA and all ports are GM, >0: Disable BMCA, the selected port is Slave port, and other ports are master port|
|SUPPORT_RUNTIME_NOTICE_CHECK| Enable this flag, then gptp stack will listener for event to trigger Message Interval Request signaling to the other side|
|CLOCK_COMPUTE_INTERVAL_MSEC| compute phase and freq every this time|
|CMLDS_MODE| In case of number of domain is 2, CMLDS_MODE must be True to apply common link delay|
|SECOND_DOMAIN_THIS_CLOCK| In case of number of domain is 2, this flag also need to be set to one to indicate software clock mode|

To access the list of these parameters, along with their descriptions and default values for gPTP, please refer to the file located at:
``<mcu_plus_sdk>/source/networking/tsn/tsn-stack/tsn_gptp/gptpconf/gptp_nonyangconfig.xml``.

### gPTP Multiple Domains

At the moment, our system supports two domains, but this feature is turned off by default. 
To turn on multiple domains, follow these steps:

- Set ``#define GPTP_MAX_DOMAINS 2`` in the ``<mcu_plus_sdk>/source/networking/tsn/tsn-stack/tsn_buildconf/sitara_buildconf.h`` file
- In the file ``<mcu_plus_sdk>/source/networking/enet/core/examples/tsn/gptp_init.c``, you will see the following settings are set:
```
#if GPTP_MAX_DOMAINS == 2
    {"CMLDS_MODE", XL4_EXTMOD_XL4GPTP_CMLDS_MODE, 1},
    {"SECOND_DOMAIN_THIS_CLOCK", XL4_EXTMOD_XL4GPTP_SECOND_DOMAIN_THIS_CLOCK, 1}
#endif
```
This will activate the second domain in the gPTP system.

### gPTP Shorter Sync Interval

By default, the gPTP Sync interval is set to 125 milliseconds (log2(0.125)=-3) which is corresponding to the default SYNC_LOG value is -3 (refer ``gptp_init.c``)

``#define SYNC_LOG -3 // Default: 125ms``

If you need a shorter Sync interval (eg: apply for Quick SYNC), you can adjust it by setting a specific value in the ``gptp_init.c`` file:

```
#define SYNC_LOG -4 // 62.5ms
#define SYNC_LOG -5 // 31.25ms
#define SYNC_LOG -6 // 15.625ms
#define SYNC_LOG -7 // 7.8125ms <- Minimum time gptp stack can support
```

As explained in previous section, the SYNC_LOG will be applied to ``ieee802-dot1as-gptp:log-sync-interval``, and the gptp stack will base on the setting to have proper SYNC interval.

This SYNC_LOG understanding is also applicable for:
- LOG_ANNOUNCE: Default 0 which mean Announce will be sent every 1s
- LOG_PDELAY: Default 0 which mean PDelay Request will be sent every 1s
- LOG_GPTP_CAPABLE: Default 3 which mean GPTP CAPABLE Signaling will be send every 8s 

### gPTP Performance optimization

Out of box configuration is not optimized for performance as the connected devices (e.g. Intel I218/219) may not be compatible with these optimal configuration.
The following configurations can be tuned for optimizing the gPTP performance:

 - ``XL4_EXTMOD_XL4GPTP_CLOCK_COMPUTE_INTERVAL_MSEC``:
    The interval in milliseconds at which the gPTP clock is computed.
    The default value is currently set to ``1000``, and has a parameter vsize of ``1`` (uint8_t).

 - ``XL4_EXTMOD_XL4GPTP_FREQ_OFFSET_UPDATE_MRATE_PPB``:
    The rate at which the frequency offset is updated.
    The default value is currently set to ``10``, and has a parameter vsize of ``4`` (uint32_t).

 - ``XL4_EXTMOD_XL4GPTP_PHASE_OFFSET_IIR_ALPHA_STABLE_VALUE``:
    The reciprocal of this is the alpha value of the IIR filter used to compute the phase offset.
    The default value is currently set to ``10``, and has a parameter vsize of ``4`` (uint32_t).

 - ``XL4_EXTMOD_XL4GPTP_FREQ_OFFSET_IIR_ALPHA_STABLE_VALUE``:
    The reciprocal of this is the alpha value of the IIR filter used to compute the frequency offset.
    The default value is currently set to ``10``, and has a parameter vsize of ``4`` (uint32_t).

e.g.
The following is an example configuration which you can use in the example to get better accuaracy if compatible with your devices in the setup.
The following configuration is tested with the Intel I210 card and in between two EVMs.

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
For all parameters related to interval, the minimum interval can be configured for gPTP is 7.8125ms. The reason for that minimum interval is the implementation uses a base timer which has minimum period is 7.8125ms. Thus, increasing the interval parameters like `sync-interval` which is not a multiple of 7.8125ms may produce inaccurate interval of sending sync message.Or reducing interval of sync message below 7.8125ms is not possible. And the same limitation applied for all remaining interval parameters.

# See Also

\ref NETWORKING
