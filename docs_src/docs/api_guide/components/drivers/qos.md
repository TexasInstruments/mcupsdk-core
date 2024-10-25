# QoS {#DRIVERS_QOS_PAGE}

[TOC]

Majority of the initiator has a dedicated QoS block to provide the configurability of the transaction characteristic,
such as orderID, priority/epriority, asel and etc. OrderID is a 4 bits value, which is associated with each transaction.
By default, all the transactions have orderID value set to 0x0. The orderID is used as a mechanism to load balance the
traffic to DDR through two parallel paths. The transactions with order ID 0-7 share one path, while transactions with 8-15
share a separate path. The OrderID value can be changed through QoS block for the initiators or through BCDMA and pktDMA
configuration.

Each transaction in the system carries 3 bits priority information. The priority information is used for cbass for
arbitration decision, which implements typical priority based round robin. Priority value 0x0 is the highest priority,
while 0x7 is the lowest priority. By default, QoS has priority value set to 0x7( lowest priority).

Refer to the QoS programming guide in the SoC Technical Reference Manual(TRM) for more details.

## Features Supported

- Setup the QoS with the given configuration data.

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

- Option to specify the QoS config generated using the "K3 Resource Partitioning" tool

## Features not Supported

NA

## Creating your own QoS config file

### Step 1: Generate a QoS config file

- QoS config file can be generated using the k3-resource-partitioning tool available in the
`${SDK_INSALL_PATH}/tools/sysfw/boardcfg/respart/@VAR_SOC_NAME_LOWER/k3-respart-tool-0.5 directory` directory.
- Open the SysConfig tool GUI from the desktop shortcut and select the software product by navigating to the k3-resource-partitioning
tool path in the SDK.
- Click on Browse button and open the existing syscfg file at `${SDK_INSTALL_PATH}/tools/sysfw/boardcfg/respart/@VAR_SOC_NAME_LOWER/k3-respart-tool-0.5/out` directory.

\imageStyle{qos_config_tool.png,width:70%}
\image html qos_config_tool.png "K3-Resource-Partitioning tool"

- Add the required number of QoS module instances and configure the parameters.
- Select a device, choose the endpoints, and select a list of channels for which QoS should be programmed.
- Note that it is possible to add multiple instances of QoS module with same device as long as the endpoints and channels do not overlap.
- The tool will generate a `@VAR_SOC_NAME_LOWER`_qos_data.c file.

\imageStyle{qos_config_save.png,width:80%}
\image html qos_config_save.png "Configure the QoS"

- The QoS data generated can to be replaced in the default qos_data.h file located in the
`${SDK_INSALL_PATH}/source/drivers/qos/v0/soc/@VAR_SOC_NAME_LOWER/` directory.
- Copy the contents of the `@VAR_SOC_NAME_LOWER`_qos_data structure in the `@VAR_SOC_NAME_LOWER`_qos_data.c file and replace it in the
gQosData structure of the qos_data.h file.
- Or you can create a new copy of qos_data.h file and replace in it.

\imageStyle{qos_config_copy.png,width:80%}
\image html qos_config_copy.png "Copy the QoS data"

- Save the generated `@VAR_SOC_NAME_LOWER`_qos_data.c in your project workspace or work area if required.

### Step 2: Add the generated QoS config file to your project

- Open SysConfig (\ref SYSCONFIG_INTRO_PAGE) for your project (typically bootloader).
- Add "QoS" as shown below

\imageStyle{qos_sysconfig.png,width:70%}
\image html qos_sysconfig.png "Add QoS via SysConfig"

- If you have created a new qos_data.h file, specify the path to your file including the filename in the sysconfig text box
  as shown above

  - Make sure to use forward slash "/" in the file path so that this will work with linux as well as windows build
  - Make sure that path to this is file set in your application include path, as needed.

- Save the sysconfig project and build your application

## Important Usage Guidelines

- Priority, orderID and asel value from the QoS block shall only be modified as part of the initialization
steps, when the SoC is idle.
- Changing the QoS block configuration during the run time is prohibited and may cause system error or other
undefined behavior. Hence it is recommended to do the QoS configuration in the bootloader(SBL).

## Example Usage

Include the below file to access the APIs
\snippet Qos_sample.c include

QoS initialization example
\snippet Qos_sample.c init

## API

\ref DRV_QOS_MODULE