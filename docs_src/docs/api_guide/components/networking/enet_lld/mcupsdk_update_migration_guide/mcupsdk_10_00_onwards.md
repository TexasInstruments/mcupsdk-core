# Version Updates from earlier SDKs to Latest {#enet_mcupsdk_10_00_update}
[TOC]

\cond SOC_AM273X
\note To migrate from SDK version 09.02 to 11.02, you must apply all the change sets described in the sections below.
\endcond

## While upgrading from 11.01 or earlier to latest SDK
### Change Set
**Change-1**: Sysconfig generated file changes<br>
**Change-2**: Enet Module interface removed

#### Change-1 Description
Enet-LLD sysconfig layer has been cleaned up to improve code readability by removing sysconfig API abstractions and statically initializing CPSW/ICSSG configurations and calling driver APIs directly.

The following new files are generated from sysconfig. Example makefiles have been updated to include these files:

| Generated File        | Description |
|:----------------------|:------------|
| ti_enet_dma_init.h    | Contains DMA structures and API definitions |
| ti_enet_dma_init.c    | Contains DMA object initialization based on the DMA channel config settings in Sysconfig GUI |
| ti_enet_init.c        | Contains global CPSW config and RM resource config initialization along with helper functions:<br>- Global RM resource config is populated from an array defined in device specific .js files<br>- Global CPSW config is populated from the Sysconfig GUI settings |

The following APIs have been removed from syscfg generated files:

| Removed API                      | Description                              |
| :--------------------------------| :----------------------------------------|
| EnetApp_enablePorts()            | Replaced by separate host/mac functions  |
| EnetApp_openDmaChannels()        | Inlined into main flow                   |
| EnetApp_openAllRxDmaChannels()   | Inlined into main flow                   |
| EnetApp_openAllTxDmaChannels()   | Inlined into main flow                   |
| EnetApp_doCpswOpen()             | No longer needed, replaced with direct call to Enet_open() |
| EnetAppUtils_initResourceConfig()| Replaced with static config              |

The following APIs have been added in syscfg generated files:

| New API                                               | Description                         |
| :-----------------------------------------------------| :-----------------------------------|
| EnetApp_enableHostPort()                              | Separate host port enable           |
| EnetApp_enableMacPort()                               | Separate MAC port enable            |
| EnetApp_getPhyAliveStatus()                           | Direct PHY status check             |
| EnetApp_updateTxChInitCfg()                           | Direct TX channel config update     |
| EnetApp_updateRxChInitCfg()                           | Direct RX channel config update     |
| EnetApp_getCpswCfg()                                  | Returns static CPSW config pointer  |
| EnetApp_cpswInitMacAddr()                             | Static MAC address init             |
| getAleModeFlags(), getAlePortMask()                   | Template helper functions           |
| getIpNxtWhitelistCount(), getIpNxtWhitelistInitArray()| Template helper functions           |

#### Change-1 Impact
- All networking examples now generate additional files that are included in makefiles - ti_enet_dma_init.c/h and ti_enet_init.c.
- Enet driver initialization has been internally refactored to reduce abstraction. Users need to rebuild their projects to get the modified template changes.

#### Change-2 Description
Enet module abstraction layer has been removed to meet functional safety standards by replacing generic `EnetMod_Handle` with specific typed handles (e.g., `CpswAle_Handle`, `Mdio_Handle`), eliminating function pointers and void pointers.
- The `void *arg` argument used in the function pointer of `EnetPhy_Mdio` structure has been changed to `Mdio_Obj*`
- The `EnetMod_Obj enetMod` member in all module structures has been removed and its individual members have been expanded inline
- Module APIs now use module-specific handles instead of the generic `EnetMod_Handle` as below:

| Module         | Impacted APIs | Old parameters | New parameters |
|:---------------|:--------------|:---------------|:---------------|
| CPSW ALE       | CpswAle_open, CpswAle_rejoin, CpswAle_ioctl,<br> CpswAle_close, CpswAle_saveCtxt, CpswAle_restoreCtxt       |`EnetMod_Handle hMod, ...`|`CpswAle_Handle hAle, ...`  |
| CPSW CPTS      | CpswCpts_open, CpswCpts_rejoin, CpswCpts_ioctl,<br> CpswCpts_close, CpswCpts_saveCtxt, CpswCpts_restoreCtxt |`EnetMod_Handle hMod, ...`  |`CpswCpts_Handle hCpts, ...`|
| CPSW HostPort  | CpswHostPort_open, CpswHostPort_rejoin, CpswHostPort_ioctl,<br> CpswHostPort_close, CpswHostPort_saveCtxt, CpswHostPort_restoreCtxt|`EnetMod_Handle hMod, ...`|`CpswHostPort_Handle hPort, ...`|
| CPSW MacPort   | CpswMacPort_open, CpswMacPort_rejoin, CpswMacPort_ioctl,<br> CpswMacPort_close, CpswMacPort_saveCtxt, CpswMacPort_restoreCtxt      |`EnetMod_Handle hMod, ...`|`CpswMacPort_Handle hPort, ...` |
| CPSW Stats     | CpswStats_open, CpswStats_rejoin, CpswStats_ioctl,<br> CpswStats_close, CpswStats_saveCtxt, CpswStats_restoreCtxt                  |`EnetMod_Handle hMod, ...`|`CpswStats_Handle hStats, ...`  |
| MDIO           | Mdio_open, Mdio_rejoin, Mdio_ioctl, Mdio_close, Mdio_saveCtxt, Mdio_restoreCtxt    |`EnetMod_Handle hMod, ...`|`Mdio_Handle hMdio, ...`   |
| ICSSG Stats    | IcssgStats_open, IcssgStats_rejoin, IcssgStats_ioctl, IcssgStats_close             |`EnetMod_Handle hMod, ...`|`IcssgStats_Handle hStats, ...`|
| ICSSG TAS      | IcssgTas_open, IcssgTas_rejoin, IcssgTas_ioctl, IcssgTas_close                     |`EnetMod_Handle hMod, ...`|`IcssgTas_Handle hTas, ...`|
| ICSSG TimeSync | IcssgTimeSync_open, IcssgTimeSync_rejoin, IcssgTimeSync_ioctl, IcssgTimeSync_close |`EnetMod_Handle hMod, ...`|`IcssgTimeSync_Handle hTimeSync, ...`|
| NullMod        | NullMod_open                                                                       |`EnetMod_Handle hMod, ... , const void *cfg`|`NullMod_Handle hNull, ... , const NullMod_Cfg *nullModCfg`|
| ^              | NullMod_rejoin, NullMod_ioctl, NullMod_close                                       |`EnetMod_Handle hMod, ...`|`NullMod_Handle hNull, ...`|
| EnetPhyMdioDflt| EnetPhyMdioDflt_isAlive, EnetPhyMdioDflt_isLinked, EnetPhyMdioDflt_readC22,<br> EnetPhyMdioDflt_writeC22, EnetPhyMdioDflt_readC45, EnetPhyMdioDflt_writeC45 |`..., void *args`|`..., Mdio_Obj *hMdio`|
| EnetPhy        | EnetPhy_open                                                                       |`..., void *mdioArgs`     |`..., Mdio_Obj *mdioArgs`  |

#### Change-2 Impact

1. All code using `EnetMod_Handle` has been replaced with module specific handle type (e.g., `CpswAle_Handle`, `Mdio_Handle`)
2. All code using the `ENET_MOD()` macro has been removed
3. All generic wrapper calls for module IOCTLs have been replaced with module-specific functions
4. PHY code that passes `void*` pointers has been updated to use `Mdio_Obj*` instead

## While upgrading from 11.00 SDK to 11.01 SDK
### Change Set
Change-1 : ENET UDMA API changes

#### Change-1 Description
The ENET UDMA API has undergone some changes, with the primary goal of enhancing overall code quality and adhering to functional safety standards. These changes aim to minimize the use of error-prone constructs and eliminate the utilization of void pointers.

Old API Header                                                                                | New API Header                                                                     |
:-------------------------------------------------------------------------------------------  | :-------------                                                                     |
1. Initialize RX channel open parameters: EnetDma_initRxChParams(void *pRxChCfg)              | EnetUdma_initRxFlowParams(EnetUdma_OpenRxFlowPrms *pRxFlowPrms)
2. Enet DMA open RX channel: EnetDma_openRxCh(EnetDma_Handle hDma, const void *pRxChCfg)      | EnetUdma_openRxFlow(EnetDma_Handle hDma,const EnetUdma_OpenRxFlowPrms *pRxFlowPrms)|
3. Enet DMA close RX channel: EnetDma_closeRxCh(EnetDma_RxChHandle hRxCh, EnetDma_PktQ *fq, EnetDma_PktQ *cq)  | EnetUdma_closeRxFlow(EnetDma_RxChHandle hRxCh,EnetDma_PktQ *fq,EnetDma_PktQ *cq)|
4. Initialize TX channel open parameters: EnetDma_initTxChParams(void *pTxChCfg)             |   EnetUdma_initTxChParams(EnetUdma_OpenTxChPrms *pTxChPrms)|
5. Enet DMA open TX channel: EnetDma_openTxCh(EnetDma_Handle hDma, const void *pTxChCfg)     | EnetUdma_openTxCh(EnetDma_Handle hDma, const EnetUdma_OpenTxChPrms *pTxChPrms) |
6. Enet DMA close TX channel: EnetDma_closeTxCh(EnetDma_TxChHandle hTxCh, EnetDma_PktQ *fq, EnetDma_PktQ *cq) | EnetUdma_closeTxCh(EnetDma_TxChHandle hTxCh, EnetDma_PktQ *fq, EnetDma_PktQ *cq)|




## While upgrading from 10.00 SDK
### Change Set
Change-1 : ENET(CPSW) and ENET(ICSS) pinmux configuration.<br>
Change-2 : Support to configure link parameters in Sysconfig GUI tool.<br>
Change-3 : ENET(CPSW) and ENET(ICSS) Example paths have been changed.<br>
Change-4 : Support to add and configure Ethernet PHY module through sysconfig-GUI tool.<br>

#### Change-1 Description
In MCU+ SDK version of 10.01, ENET(CPSW) and ENET(ICSS) related examples showcasing various features have been moved from ${MCU_PLUS_SDK_PATH}/examples/networking to ${MCU_PLUS_SDK_PATH}/source/networking/enet/core/examples

#### Change-1 Impact
The above change makes older build commands to build Networking examples no longer supported, and it has to be updated with correct path.

#### Change-1 Solution
Please use "make help DEVICE=<dev_name>" to get the correct location and build commands for the examples.
\note For Windows , please use "gmake" instead of "make"

#### Change-2 Description
In MCU+ SDK version of 10.01, support to add custom PHY and configure it through sysconfig tool has been added, and this needs updates in corresponding example.syscfg file to be compatible with latest changes

#### Change-2 Impact
This will need addition of the corresponding sysconfig tool changes to add the ethernet phy module and configure it.

#### Change-2 Solution
- Refer \ref phy_integration_guide_top

## While upgrading from 09.02 SDK or earlier
### Change-1 Description
In MCU+ SDK version of 10.00, ENET(CPSW) and ENET(ICSS) component in Sysconfig GUI has been updated with fixes to correct MII/RMII mode for ICSSG and CPSW Resp..
Additionally support added to release unused pins based on number of ports were enabled in CPSW.

### Change-1 Impact
The above described change makes any older 'example.syscfg' generated using SDK version of 09.00 or earlier not compatible with SDK version 10.00 or later. This incompatibility is seen only when either ENET(CPSW) or ENET(ICSS) is added in your application/example.If neither of the components are added, you may ignore this change and there is no impact.
User will see the error in sysconfig gui tool when the old example.syscfg file is opened in updated version of mcu plus sdk.

### Change-1 Solution
#### Option 1:
If it is possible to create an example.syscfg as per user configurations, please generate a new example.syscfg from any SDK example and change the configuration as per your need. This is the safest option.

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