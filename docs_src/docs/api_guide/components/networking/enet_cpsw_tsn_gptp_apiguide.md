# Ethernet TSN and gPTP Stack - API and Integration Guide {#ENET_CPSW_TSN_GPTP}

[TOC]

# Pre-requisites
Reader are expected to have basic knowledge on below IEEE specifications
- Standard ethernet (IEEE 802.1)
- Timing and Synchronization for Time-Sensitive Applications - gPTP (IEEE 802.1AS-2020)
- Forwarding and Queuing Enhancements for Time-Sensitive Streams (IEEE 802.1Qav)
- Enhancements for Scheduled Traffic (IEEE 802.1Qbv)
- Frame Preemption (IEEE 802.1Qbu)


# Introduction
This guide is intended to enhance user's understanding of the TSN stack and provide guidance on how to seamlessly integrate TSN modules into their own applications.

# Demo and Examples
\ref EXAMPLES_ENET_CPSW_TSN_GPTP_BRIDGE
\ref EXAMPLES_ENET_CPSW_TSN_GPTP_TR
\ref EXAMPLES_ENET_CPSW_TSN_GPTP_TT

# TSN Stack

## Compilation
TSN stack library is pre-built in SDK and located in ``<mcu_plus_sdk>/examples/networking/tsn`` directory as .lib files. Any changes in the stack source code needs rebuild of library.
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

## Stack Initialization

A reference example of the TSN stack initialization can be found in ``<mcu_plus_sdk>/examples/networking/tsn/tsninit.c``.
Prior to any module calls, it is necessary to initialize the unibase library once.
This can be achieved by invoking the ``EnetApp_initTsnByCfg()`` function.

## Logging

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

## Starting uniconf and gPTP

The gPTP functionality operates in a separate task, alongside the universal configuration daemon "uniconf".
To start these tasks, use the ``EnetApp_startTsn()`` in tsninit.c as reference .

This function will start the uniconf and gPTP tasks.

## TSN Deinitialization

To deinitialize the TSN modules, you can invoke the ``EnetApp_stopTsn();`` and ``EnetApp_deInitTsn();`` functions.

## gPTP Multiple Domains

At the moment, our system supports two domains, but this feature is turned off by default. 
To turn on multiple domains, follow these steps:

- Set ``#define GPTP_MAX_DOMAINS 2`` in the ``<mcu_plus_sdk>/source/networking/tsn/tsn-stack/tsn_buildconf/sitara_buildconf.h`` file
- In the file ``<mcu_plus_sdk>/examples/networking/tsn/gptp_init.c``, you will see the following settings are set:
```
#if GPTP_MAX_DOMAINS == 2
    {"CMLDS_MODE", XL4_EXTMOD_XL4GPTP_CMLDS_MODE, 1},
    {"SECOND_DOMAIN_THIS_CLOCK", XL4_EXTMOD_XL4GPTP_SECOND_DOMAIN_THIS_CLOCK, 1}
#endif
```
This will activate the second domain in the gPTP system.

## gPTP Shorter Sync Interval

By default, the gPTP Sync interval is set to 125 milliseconds. 
If you need a shorter Sync interval, you can adjust it by setting a specific value in the ``sitara_buildconf.h`` file:

```
/* Interval timeout in nanoseconds used to generate timers in GPTP. 
 * Supported values are 125, 62.5, 31.25, 15.625 and 7.8125 milliseconds. */
#define GPTPNET_INTERVAL_TIMEOUT_NSEC 15625000u
```

``GPTPNET_INTERVAL_TIMEOUT_NSEC`` must be equal to or less than the desired Sync interval time. 
For instance, if you want a Sync interval time of 31.25 milliseconds, set ``GPTPNET_INTERVAL_TIMEOUT_NSEC`` to 31.25, 15.625, or 7.8125 milliseconds.
Be aware that decreasing ``GPTPNET_INTERVAL_TIMEOUT_NSEC`` will increase CPU load. 
Additionally, adjust the ``log-sync-interval`` in the standard yang config by referring to the ``gptp_init.c`` file.
For example, to set the Sync interval to 31.25 milliseconds, set ``log-sync-interval`` to -5.

# Integration
## Source integration

To integrate the TSN stack into your application, follow these steps:

- Initialize Enet LLD and setup board dependencies.  In the TSN example application,
  the initialization routines can be found at ``<mcu_plus_sdk>/examples/networking/tsn/tsnapp_cpsw_main.c.c``,
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

## gPTP Yang Config Parameters

This section describes the standard Yang parameters utilized for gPTP.
To access the list of these parameters along with their default values for gPTP, please refer to the file located at:
``<mcu_plus_sdk>/source/networking/tsn/tsn-stack/tsn_gptp/gptpconf/gptp-yangconfig.xml``.

For detailed descriptions of each parameter, please refer to the following Yang files:
https://github.com/YangModels/yang/blob/main/standard/ieee/draft/1588/ieee1588-ptp.yang
https://github.com/YangModels/yang/blob/main/standard/ieee/draft/802.1/ASdn/ieee802-dot1as-ptp.yang

To modify the default value of the Yang parameters, please refer to the ``gptp_yang_config()``
function in the ``tsninit.c`` file. This function will demonstrate how to update the parameter values at runtime.

## gPTP Non-Yang Config Parameters

In addition to the standard Yang parameters, gPTP also includes a set of non-Yang configuration parameters
that are specific to its implementation. To access the list of these parameters, along with their descriptions
 and default values for gPTP, please refer to the file located at:
``<mcu_plus_sdk>/source/networking/tsn/tsn-stack/tsn_gptp/gptpconf/gptp_nonyangconfig.xml``.

To modify the default value of the Non-Yang parameters, please refer to the ``gptp_nonyang_config()`` function
in the ``tsninit.c`` file. This function will invokes ``gptpgcfg_set_item()`` to configure each params:

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
- As a reference, please consult the ``<mcu_plus_sdk>/examples/networking/tsn/tsninit.c``
file.

- The configuration must be done before calling the ``gptpman_run()`` function
  inside the ``tsninit.c`` file, and it should follow the the gptpgcfg_init function call.

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
