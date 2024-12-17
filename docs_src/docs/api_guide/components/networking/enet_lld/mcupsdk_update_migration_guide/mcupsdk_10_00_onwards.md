# Version Updates from earlier SDKs to Latest {#enet_mcupsdk_10_00_update}

[TOC]
## Change Set
Change-1 : ENET(CPSW) and ENET(ICSS) pinmux configuration.
Change-2 : Support to configure link parameters in Sysconfig GUI tool.
Change-3 : ENET(CPSW) and ENET(ICSS) Example paths have been changed.
Change-4 : Support to add and configure Ethernet PHY module through sysconfig-GUI tool.

## While upgrading from 10.00 SDK
### Change-1 Description
In MCU+ SDK version of 10.01, ENET(CPSW) and ENET(ICSS) related examples showcasing various features have been moved from ${MCU_PLUS_SDK_PATH}/examples/networking to ${MCU_PLUS_SDK_PATH}/source/networking/enet/core/examples

### Change-1 Impact
The above change makes older build commands to build Networking examples no longer supported, and it has to be updated with correct path.

### Change-1 Solution
Please use "make help DEVICE=<dev_name>" to get the correct location and build commands for the examples.
\note For Windows , please use "gmake" instead of "make"
 
### Change-2 Description
In MCU+ SDK version of 10.01, support to add custom PHY and configure it through sysconfig tool has been added, and this needs updates in corresponding example.syscfg file to be compatible with latest changes

### Change-2 Impact
This will need addition of the corresponding sysconfig tool changes to add the ethernet phy module and configure it.

### Change-2 Solution
- Refer \ref phy_integration_guide_top

## While upgrading from 09.02 SDK or earlier
### Change-1 Description
In MCU+ SDK version of 10.00, ENET(CPSW) and ENET(ICSS) component in Sysconfig GUI has been updated with fixes to correct MII/RMII mode for ICSSG and CPSW Resp..
Additonally support added to release unused pins based on number of ports were enabled in CPSW.

### Change-1 Impact
The above described change makes any older 'example.syscfg' generated using SDK version of 09.00 or earlier not compatible with SDK version 10.00 or later. This incompatibility is seen only when either ENET(CPSW) or ENET(ICSS) is added in your application/example.If neither of the components are added, you may ignore this change and there is no impact.
User will see the error in sysconfig gui tool when the old example.syscfg file is opened in updated version of mcu plus sdk.

### Change-1 Solution
#### Option 1:
If is possible to create a example.syscfg as per user configurations, please generate a new example.syscfg from any SDK example and change the configration as per your need. This is the safest option.
 
#### Option 2:
Sometimes, it is not easy to get a new example.syscfg that matches the older example.syscfg. In those case, please follow the below steps

- Open example.syscfg file in any text editor (avoid notepad or word)
- Find and replace 'enet_cpsw1.pinmux[0].$name              = "ENET_CPSW_PINMUX0";' with 'enet_cpsw1.$name              = "CONFIG_ENET_CPSW0";'
- Find and replace 'enet_cpsw1.pinmux[0].' with 'enet_cpsw1.'  (note '.' at the end of the strings)
- Find and replace 'enet_cpsw1.macport1LoopbackEn          = true;' with 'enet_cpsw1.macport1LoopbackMode  = "LOOPBACK_MODE_MAC";'

Save the modifications. Then open sysconfig GUI tool with the modifications, save and close the tool.
 
### Change-2 Description
In MCU+ SDK version of 10.00, parameters that configures link capabilities for auto negotiation such as speed and duplexity is added into syscfg GUI.

### Change-2 Impact
'EnetApp_initLinkArgs' function that is defined in the user applicaiton code is no longer called. Instead, a new function EnetApp_initLinkArgs is defined.

### Change-2 Solution
- User shall open delete their locally defined 'EnetApp_initLinkArgs' and instead let sysconfig tool handle it via generated code.
- For CPSW users: open sysconfig gui tool and navigate to ENET(CPSW)->MACPORT configuration-> Mac Port 1/2 . Set the link speed capbility and link duplexity capability here. Keep both as Auto to use the default settings.
- For ICSSG user: open sysconfig gui tool and navigate to ENET(ICSSG) and configure link speed capabilities and link duplexity capabilities as your user configuration. Keep it as Auto to use the default settings.