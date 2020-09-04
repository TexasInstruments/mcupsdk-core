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

The top-layer of PHY driver is located at `<ENET_LLD>/src/phy/enetphy.c`. This
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
supported in the Enet LLD. The `EnetPhy_Drv` structure is defined as the
interface that device specific drivers must implement.  Since this structure is
not exposed to the application, its scope is internal to the Enet LLD.

![Ethernet PHY Driver Interface]
 (EnetPhy_Drv.png "Ethernet PHY Driver Interface")

The members of the `EnetPhy_Drv` structure can be mandatory or optional in
nature. Optional members can be set to NULL if the PHY doesn't provide an
implementation for them. The list below provides a description of the purpose
of each `EnetPhy_Drv` member.

- **name**: Driver name.
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

The current version of Enet LLD includes the following PHY drivers:
- Generic PHY driver.
- TI [DP83867](http://www.ti.com/lit/ds/symlink/dp83867cr.pdf) RGMII PHY driver.
- TI [DP83869](https://www.ti.com/lit/ds/symlink/dp83869hm.pdf) RGMII PHY driver.
- TI [DP83822](https://www.ti.com/lit/ds/symlink/dp83822i.pdf) RMII PHY driver.
- VSC8514 QSGMII PHY driver.

The generic PHY driver is a special case because its implementation is limited
to IEEE-Standard MII registers. Reuse of PHY generic function by other
device-specific drivers is possible when their `EnetPhy_Drv` implementation
doesn't deviate from standard. The diagram in figure below shows the reuse of
extended register read/write functions by the DP83867 driver.

![Ethernet PHY Driver Hierarchy]
 (EnetPhy_Drv_Hierarchy.png "Ethernet PHY Driver Hierarchy")

Device specific drivers can be found at `<ENET_LLD>/src/phy/*`.


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
static EnetPhyDrv_Handle gEnetPhyDrvs[] =
{
    &gEnetPhyDrvVsc8514,   /* VSC8514 */
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

- Create the public PHY specific header file at `<ENET_LLD>/include/phy`.
   + This header file should have the device extended configuration
     structure definition (if applicable) as well as auxiliary structures or
     enumerations.
  ![](MyPhy_h.png)
- Create new source file and private header file (if needed) for the new PHY
  driver at `<ENET_LLD>/src/phy`.
  ![](MyPhy_c.png)
- Declare a global structure of type `EnetPhy_Drv`, but don't make it static.
  This variable will be later accessed as an extern symbol by the Ethernet PHY
  driver.
- Initialize `EnetPhy_Drv` structure with the function pointers of the device
  specific implementation.
   + Look for reuse of PHY generic functions if a device specific implementation
     is not needed.

\code{.c}
EnetPhy_Drv gEnetPhyDrvMyDrv =
{
    .name               = "My PHY driver",
    .isPhyDevSupported  = MyPhy_isPhyDevSupported,
    .isMacModeSupported = MyPhy_isMacModeSupported,
    .config             = MyPhy_config,
    .reset              = MyPhy_reset,
    .isResetComplete    = MyPhy_isResetComplete,
    .runComplianceTest  = NULL,
    .readExtReg         = GenericPhy_readExtReg,
    .writeExtReg        = GenericPhy_writeExtReg,
    .printRegs          = MyPhy_printRegs,
};
\endcode

- Declare the variable of type `EnetPhy_Drv` as extern in main PHY driver
  located at `<ENET_LLD>/src/phy/enetphy.c`. Also, add it to the `gEnetPhyDrvs`
  array.

\code{.c}
extern EnetPhy_Drv gEnetPhyDrvGeneric;
extern EnetPhy_Drv gEnetPhyDrvDp83822;
extern EnetPhy_Drv gEnetPhyDrvDp83867;
extern EnetPhy_Drv gEnetPhyDrvDp83869;
extern EnetPhy_Drv gEnetPhyDrvVsc8514;
extern EnetPhy_Drv gEnetPhyDrvMyDrv;

static EnetPhyDrv_Handle gEnetPhyDrvs[] =
{
    &gEnetPhyDrvMyDrv,     /* My PHY driver */
    &gEnetPhyDrvVsc8514,   /* VSC8514 */
    &gEnetPhyDrvDp83822,   /* DP83822 */
    &gEnetPhyDrvDp83867,   /* DP83867 */
    &gEnetPhyDrvDp83869,   /* DP83869 */
    &gEnetPhyDrvGeneric,   /* Generic PHY - must be last */
};
\endcode

- Add the PHY source file to `SRCS_COMMON` in the Ethernet PHY driver makefile
  locate at `<ENET_LLD>/src/phy/makefile`.

\code
SRCS_COMMON += enetphy.c generic_phy.c
SRCS_COMMON += dp83869.c dp83867.c dp83822.c vsc8514.c
\endcode

- PHY driver directory is already part of the `SRCDIR`, so just the source
  name needs to be added.


# Appendix {#enetphy_guide_appendix}

## Appendix A {#enetphy_guide_appendix_a}

Detailed view of the PHY state machine.

![Detailed View of PHY State Machine]
 (EnetPhy_FSM.png "Detailed View of PHY State Machine")
