Ethernet PHY Integration Guide {#enetphy_guide_top}
=====================

[TOC]

# Introduction {#enetphy_guide_intro}

The Ethernet PHY driver is currently part of the Enet low-level driver (LLD),
it's dedicated to Ethernet PHY management. It implements a state machine
required to handle the lifecycle of PHYs, from initialization to link
establishment.

The PHY submodule interacts with an underlying MDIO hardware through a simple
MDIO driver abstraction (see \ref EnetPhy_Mdio) in order to perform operations
like detecting alive and/or linked PHYs, and for PHY register accesses. The
relationship between PHY, MDIO and Enet LLD integration layer is shown below.

![Enet Low-Level Driver Block Diagram]
 (EnetLLD_Diagram.png "Enet Low-Level Driver Block Diagram")

Currently, the PHY driver supports only Clause-22 devices. Clause-45 devices
are not supported, but read and write helper functions are provided.


# PHY Driver {#enetphy_guide_driver}

The top-layer of PHY driver is located at `source/networking/enet/core/src/phy/enetphy.c`. This
layer implements the basic APIs that are needed to communicate the driver,
such as `EnetPhy_open()`, `EnetPhy_tick()`, `EnetPhy_close()`, etc.

The Enet LLD is capable of supporting multiple PHYs running simultaneously;
each PHY has its own driver instance, its own state machine and hence will
follow independent lifecycle from other PHYs.

The lifecycle of the PHYs is handled by the PHY driver via a state machine
implementation. This state machine is composed of the following states:

- **FINDING**: The driver will remain in this state until the PHY is detected
  as alive and a device-specific PHY driver has been bound to it.
- **FOUND**: The driver will initiate (soft) reset if requested in PHY
  configuration.
- **RESET_WAIT**: The driver will remain in this state while waiting for reset
  to complete.
- **ENABLE**: The driver will put the PHY in normal mode, perform PHY
  device-specific extended configuration and get common capabilities supported
  by the MAC port and local PHY device. Depending on the requested mode
  (auto-negotiation or manual), the PHY will either restart auto-negotiation or
  manually configure speed and duplexity.
- **LOOPBACK**: Last state if PHY loopback is enabled.
- **NWAY_START**: The driver will remain in this state while waiting for
  auto-negotiation to start.
- **NWAY_WAIT**: The driver will remain in this state while waiting for
  auto-negotiation to complete.
- **LINK_WAIT**: The driver will remain in this state while waiting for link
  up.
- **LINKED**: The PHY will remain in this state until the link is down,
  i.e. cable disconnection.

The diagram in the figure below provides a simplified view of the state
transitions of the PHY state machine.

![Simplified View of PHY State Machine]
 (EnetPhy_FSM_SimplifiedView.png "Simplified View of PHY State Machine")

Refer to @ref enetphy_guide_appendix_a for a more detailed view of the PHY state machine.


## Device-Specific Drivers {#enetphy_guide_device_specific}

The PHY driver model has been designed to partition device-specific operations
from device-agnostic common operations. The former are implemented by PHY device
specific drivers, while the latter can be carried out directly by the main PHY
driver.

This model facilitates the addition of new drivers for PHY devices not yet
supported in the Enet LLD. The `Phy_DrvObj_t` structure is defined as the
interface that device specific drivers must implement.  Since this structure is
not exposed to the application, its scope is internal to the Enet LLD.

![Ethernet PHY Driver Interface]
 (EnetPhy_Drv.png "Ethernet PHY Driver Interface")

The members of the `Phy_DrvObj_t` structure can be mandatory or optional in
nature. Optional members can be set to NULL if the PHY doesn't provide an
implementation for them. The list below provides a description of the purpose
of each `Phy_DrvObj_t` member.

- **name**: Driver name.
- **bind**: The main PHY driver will call this function to bind the driver handle and register access functions to specific PHY device.
- **isPhyDevSupported()**: Function pointer used by the main PHY driver to check
  if this device specific driver support a given PHY hardware identified by
  its version (OUI). This function is used during PHY device to driver binding.
- **isMacModeSupported()**: The main PHY driver will call this function to check
  if the device specific driver supports the requested MAC mode (MII, RMII,
  RGMII, SGMII, etc).
- **config()**: The main PHY driver will call this function to allow the driver
  to perform any device-specific extended configuration. This function is
  optional and will not be called if the driver sets the function pointer to
  `NULL`.
- **reset()**: Device specific reset. This function is optional and will not be
  called if the driver sets the function pointer to `NULL`.
- **isResetComplete()**: The main PHY driver will call this function to check
  if the reset operation triggered by `reset()` function has completed.
  If `reset()` is provided, `isResetComplete()` must be provided as well.
- **readExtReg()**: The main PHY driver will call this function to read extended
  registers.
- **writeExtReg()**: The main PHY driver will call this function to write extended
  registers.
- **printRegs()**: The main PHY driver will call this function when
  EnetPhy_printRegs() is called. This function is optional and will not be called
  if the driver sets the function pointer to `NULL`.

 The below functions are optional and can only be supported when the PHY has a built-in PTP clock.
- **adjPtpFreq()**: The main PHY driver will call this function when EnetPhy_adjPtpFreq() is called to adjust PTP clock frequency.
- **adjPtpPhase()**: The main PHY driver will call this function when EnetPhy_adjPtpPhase() is called to adjust PTP clock phase.
- **getPtpTime()**: The main PHY driver will call this function when EnetPhy_getPtpTime() is called to get current PHY PTP clock time.
- **setPtpTime()**: The main PHY driver will call this function when EnetPhy_setPtpTime() is called to set PHY PTP clock time.
- **getPtpTxTime()**: The main PHY driver will call this function when EnetPhy_getPtpTxTime() is called to get PHY PTP TX packet timestamp.
- **getPtpRxTime()**: The main PHY driver will call this function when EnetPhy_getPtpRxTime() is called to get PHY PTP RX packet timestamp.
- **waitPtpTxTime()**: The main PHY driver will call this function when EnetPhy_waitPtpTxTime() is called to add PHY PTP TX packet info to a waiting TX timestamp list.
- **procStatusFrame()**: The main PHY driver will call this function when EnetPhy_procStatusFrame() is called to process PHY status frame.
- **getStatusFrameEthHeader()**: The main PHY driver will call this function when EnetPhy_getStatusFrameEthHeader() is called to get the Ethernet header of the PHY status frame.
- **enablePtp()**: The main PHY driver will call this function when EnetPhy_enablePtp() is called to enable/disable the PHY PTP module.
- **tickDriver()**: The main PHY driver will call this function when EnetPhy_tickDriver() is called to provide timer tick to the driver.
- **enableEventCapture()**: The main PHY driver will call this function when EnetPhy_enableEventCapture() is called to enable/disable an event capture on a PHY GPIO pin. This function can only be supported when the built-in PTP clock supports event capture.
- **enableTriggerOutput()**: The main PHY driver will call this function when EnetPhy_enableTriggerOutput() is called to enable/disable trigger output on a GPIO pin. This function can only be supported when the built-in PTP clock supports trigger output.
- **getEventTs()**: The main PHY driver will call this function when () is called when EnetPhy_getEventTs() is called to get event timestamp.

The current version of Enet LLD includes the following PHY drivers:
- Generic PHY driver.
- TI [DP83867](http://www.ti.com/lit/ds/symlink/dp83867cr.pdf) RGMII PHY driver.
- TI [DP83869](https://www.ti.com/lit/ds/symlink/dp83869hm.pdf) RGMII PHY driver.
- TI [DP83822](https://www.ti.com/lit/ds/symlink/dp83822i.pdf) RMII PHY driver.
- TI [DP83826](https://www.ti.com/lit/ds/symlink/dp83826e.pdf) RMII PHY driver.
- TI [DP83TG720](https://www.ti.com/lit/ds/symlink/dp83tg720s-q1.pdf) Automotive PHY driver.
- TI [DP83TG721](https://www.ti.com/lit/ds/symlink/dp83tg721s-q1.pdf) Automotive PHY driver.
- TI [DP83TC812](https://www.ti.com/lit/ds/symlink/dp83tc812r-q1.pdf) Automotive PHY driver.

The generic PHY driver is a special case because its implementation is limited
to IEEE-Standard MII registers. Reuse of PHY generic function by other
device-specific drivers is possible when their `Phy_DrvObj_t` implementation
doesn't deviate from standard. The diagram in figure below shows the reuse of
extended register read/write functions by the DP83867 driver.

![Ethernet PHY Driver Hierarchy]
 (EnetPhy_Drv_Hierarchy.png "Ethernet PHY Driver Hierarchy")

Device specific drivers can be found at `source/board/ethphy/enet/rtos_drivers/src/*`.

# Custom Board Support {#CustomBoardPhySupport}
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - ----------
The MCU+SDK enet driver supports a set of boards for each SoC out of the box
- Refer MCU+SDK release notes for platforms/board supported for each SoC

The board specific portion of the enet code is auto generated in the file ti_board_config.c for supported boards

For porting enet based applications to custom board the following need to be done:
Enable "Custom Board" syscfg option
  \imageStyle{CustomBoardSyscfg.png,width:30%}
  \image html CustomBoardSyscfg.png

- Enabling “Custom Board” will prevent auto generation of board specific code.
- A C file will have to be then written that is specific to the board.

### Board config C file
The board specific file should contain the following
- **const EnetPhy_DrvInfoTbl gEnetPhyDrvTbl**: This is a table of ENET PHY drivers supported on the board. 
  Refer Enet custom PHY integration guide for details on how to populate this table @ref enetphy_guide_top
- **EnetBoard_setupPorts()**: This function should setup any board level muxes and configure any SoC level 
  RGMII internal delay/ RMII configuration for the specific port. 
    + Refer API documentation for mcu_plus_sdk/source/networking/enet/utils/include/enet_board.h for details of function and arguments
- **EnetBoard_getPhyCfg()**: This function should return the ETHPHY specific configuration for a given port including any extended phy 
  configuration
    + Refer API documentation for mcu_plus_sdk/source/networking/enet/utils/include/enet_board.h for details of function and arguments
- **EnetBoard_getMacAddrList()**: This function should populate any board specific MAC addresses that are available on board eeprom.
  If the board does not have any board specific macAddresses this function should set argument `*pAvailMacEntries = 0`
- **EnetBoard_getId()**: This function should return the board id. This is not used anywhere outside this file so the board id returned 
  will depend on the implementation of EnetBoard_setupPorts()/EnetBoard_getPhyCfg()  for the custom board if it refers the boardId to 
  determine PHY config/setup ports.
- Refer mcu_plus_sdk/examples/networking/enet_layer2_multi_channel
    + enet_custom_board_config.c for example illustrating custom board integration

## PHY to Driver Binding {#enetphy_guide_binding}

The PHY-to-driver binding is the process of selecting the best driver for a PHY
device based on the device's unique identifier. This process is done by the
main PHY driver upon *alive* detection of a PHY device and takes place in the
*FINDING* state.

The device unique identifier is read from PHYIDR1 and PHYIDR2 registers and
populated into a structure of type #EnetPhy_Version.

![Ethernet PHY Version]
 (EnetPhy_Version.png "Ethernet PHY Version")

The main PHY driver has a `gEnetPhyDrvs` array which contains all device
specific drivers that are registered and that will participate in the search
for the best driver for the PHY that had been recently discovered.

\code{.c}
static const EthPhyDrv_If gEnetPhyDrvs[] =
{
    &gEnetPhyDrvDp83822,   /* DP83822 */
    &gEnetPhyDrvDp83867,   /* DP83867 */
    &gEnetPhyDrvDp83869,   /* DP83869 */
    &gEnetPhyDrvGeneric,   /* Generic PHY - must be last */
};
\endcode

In the search process, the main PHY driver will call the `isPhyDevSupported()`
function of each registered driver. The drivers will use the #EnetPhy_Version
which is passed as an argument in order to determine whether the device is
supported or not. If it is, `isPhyDevSupported()` must return `true` and the
search process ends. If not, the search continues with the next registered
device.

The generic PHY (which must be the last one in the `gEnetPhyDrvs` array) will
be bound to the device if no other driver can support a given PHY, but the
PHY full functionality can't be guaranteed.


# Implementing a New PHY Driver {#enetphy_guide_implementing}

The following list of steps is provided as guideline when adding a new PHY
driver for a device which is not supported by Enet LLD.

- Select "CUSTOM" (ETHPHY (ENET CPSW/ICSSG) -> ETHPHY Device) option in Sys-Cfg gui. This will modify the auto-generated ti_board_config.c file based on the "Custom Device Name" specified in the GUI.
- Create the new source file, public PHY specific header file and private header file (if needed) along with the main.c file in the example folder.
   + This header file should have the device extended configuration
     structure definition (if applicable) as well as auxiliary structures or
     enumerations.

  \imageStyle{MyEthphy.png,width:30%}
  \image html MyEthphy.png

- In the source file, declare a global structure of type `Phy_DrvObj_t`, but don't make it static.
  This variable will be later accessed as an extern symbol by the Ethernet PHY
  driver.
- Initialize `Phy_DrvObj_t` structure with the function pointers of the device
  specific implementation.
   + Look for reuse of PHY generic functions if a device specific implementation
     is not needed.

\code{.c}
Phy_DrvObj_t gEnetPhyDrvMyethphy =
{
    .fxn =
    {
        .name               = "Myethphy", /*PHY driver name*/
        .bind               = MyEthphy_bind,
        .isPhyDevSupported  = MyEthphy_isPhyDevSupported,
        .isMacModeSupported = MyEthphy_isMacModeSupported,
        .config             = MyEthphy_config,
        .reset              = MyEthphy_reset,
        .isResetComplete    = MyEthphy_isResetComplete,
        .readExtReg         = GenericPhy_readExtReg,
        .writeExtReg        = GenericPhy_writeExtReg,
        .printRegs          = MyEthphy_printRegs,
        /*Below functions can only be supported when the PHY has a built-in PTP clock.*/
        .adjPtpFreq              = NULL,    /*adjust PTP clock frequency*/
        .adjPtpPhase             = NULL,    /*adjust PTP clock phase*/
        .getPtpTime              = NULL,    /*get current PHY PTP clock time*/
        .setPtpTime              = NULL,    /*set PHY PTP clock time*/
        .getPtpTxTime            = NULL,    /*get PHY PTP TX packet timestamp*/
        .getPtpRxTime            = NULL,    /*get PHY PTP RX packet timestamp*/
        .waitPtpTxTime           = NULL,    /*add PHY PTP TX packet info to a waiting TX timestamp list*/
        .procStatusFrame         = NULL,    /*process PHY status frame*/
        .getStatusFrameEthHeader = NULL,    /*get the Ethernet header of the PHY status frame*/
        .enablePtp               = NULL,    /*enable/disable the PHY PTP module*/
        .tickDriver              = NULL,    /*provide timer tick to the driver*/
        .enableEventCapture      = NULL,    /*enable/disable an event capture on a PHY GPIO pin*/
        .enableTriggerOutput     = NULL,    /*enable/disable trigger output on a GPIO pin*/
        .getEventTs              = NULL,    /*get event timestamp*/
    }
};
\endcode

- **When using CCS to build**, add the source file and header files to the CCS example.
    + Select the file operation as "Link to files" and select yes for Adjust Compiler Include-Paths

  \imageStyle{CustomEthphy.png,width:50%}
  \image html CustomEthphy.png

- **When using makefiles to build**, add the PHY source file to "FILES_common" and header file path to "INCLUDES_common" in the example makefile.

\code
FILES_common := \
  l2_cpsw_main.c \
  main.c \
  myethphy.c \    /*New PHY source file*/

...

INCLUDES_common := \
  -I../ \         /*Path to PHY header files*/
  -I${CG_TOOL_ROOT}/include/c \
  -I${MCU_PLUS_SDK_PATH}/source \

...
\endcode 

# MAC-to-MAC (NO-PHY mode) {#mac_to_mac_mode}


# Appendix {#enetphy_guide_appendix}

## Appendix A {#enetphy_guide_appendix_a}

Detailed view of the PHY state machine.

![Detailed View of PHY State Machine]
 (EnetPhy_FSM.png "Detailed View of PHY State Machine")
