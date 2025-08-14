MDIO PHY Register Access - Clause45 Support {#enet_mdio_clause45_support}
============================================

# Overview

[TOC]

This document describes how to enable and utilize Clause 45 MDIO functionality for PHY register access. Clause 45 provides expanded addressing capabilities for modern PHY devices with extended register sets.

# Background

## Clause 22 v/s Clause 45

  - Clause 22(IEEE802.3ah):
    - Limited to 32 registers per PHY device (5-bit register address)
    - Simple addressing scheme: PHY address (5 bits) + Register address (5 bits)
    - Cannot directly access extended registers
  - Clause 45(IEEE802.3ae):
    - Supports up to 65,536 registers (16-bit register address)
    - Adds Management/MDIO Manageable Device (MMD) field to enable multiple sub-devices within a PHY
    - Two-part addressing scheme: first address the MMD, then access the register
    - Better suited for modern PHY devices with complex functionality

## MMD (Management/MDIO Manageable Device) : Required for Clause45 support

  The MMD field identifies specific device types within a PHY:
  - MMD 1: PMA/PMD (Physical Medium Attachment/Physical Medium Dependent)
  - MMD 2: WIS (WAN Interface Sublayer)
  - MMD 3: PCS (Physical Coding Sublayer)
  - MMD 7: Auto-Negotiation
  - MMDs 30-31: Vendor-specific functions

# Implementation Requirements

  To enable Clause 45 support, the following are required:

  1. Verify hardware capabilities:
    - The MDIO should either be configured in MDIO_MODE_STATE_CHANGE_MON or MDIO_MODE_NORMAL mode of operation
    - Check ENET_CFG_MDIO_CLAUSE45 feature flag support in the file: `mcu_plus_sdk/source/networking/enet/soc/<soc_name>/enet_soc_cfg.h`
  2. API modifications:
    - Use C45-specific APIs instead of C22 APIs
    - Add MMD selection to register access calls

# Implementation Steps

## Step-1: Modify PHY Driver code with the MMD register as an input argument
\note PHY datasheet will specify which MMD values to use for accessing specific registers. Always refer to the PHY documentation for the correct MMD values.

\code
typedef struct
{
    int32_t (*EnetPhy_readReg)(void* pArgs, uint8_t mmd, uint32_t reg, uint16_t *val);

    int32_t (*EnetPhy_writeReg)(void*  pArgs, uint8_t mmd, uint32_t reg, uint16_t val);

    int32_t (*EnetPhy_rmwReg)(void*  pArgs, uint8_t mmd, uint32_t reg, uint16_t mask,
            uint16_t val);

    /*!Argument that needs to be passed to the above callbacks */
    void* pArgs;

} Phy_RegAccessCb_t;
\endcode

## Step-2: Update EnetPhy Configuration

To ensure your PHY driver properly uses Clause 45 operations, enable the following changes in `mcu_plus_sdk/source/networking/enet/core/src/phy/enetphy.c`:

1. Enable the binding for C45 PHY driver functions with EnetPhy module. 
    \code
     Phy_RegAccessCb_t regAccess =
                    {
                            .EnetPhy_readReg = EnetPhy_readC45Reg,
                            .EnetPhy_writeReg = EnetPhy_writeC45Reg,
                            .EnetPhy_rmwReg = EnetPhy_rmwC45Reg,
                            .pArgs = (void *) hPhy,
                    };
    \endcode

2. Update EnetPhy read/write/rmw access functions to use Clause 45 APIs:
    \code
    // Replace EnetPhy_readReg function call with EnetPhy_readC45Reg call:
    //EnetPhy_readReg(hPhy, reg, &val);
    EnetPhy_readC45Reg(hPhy, mmd, reg, &val);
    \endcode
    \code
    // Replace EnetPhy_writeReg funcation call with EnetPhy_writeC45Reg call
    //EnetPhy_writeReg(hPhy, reg, val);
    EnetPhy_writeC45Reg(hPhy, mmd, reg, val);
    \endcode
    \code
    //EnetPhy_rmwReg(hPhy, reg, val);
    EnetPhy_rmwC45Reg(hPhy, mmd, reg, val);
    \endcode

3. Make sure to build libraries before building the Clause45 support examples.
    To rebuild the libraries, do below,
    \code
    cd ${SDK_INSTALL_PATH}
    gmake -s libs PROFILE=release
    \endcode

## Step-3: Common PHY registers access, such as PHY alive, PHY link status, etc uses Clause 22 in order to suport backward compatibility. Replace them with Clause 45 support

\code
// Reading a register using Clause 45
int32_t status;
//status = hMdio->readC22(phyGroup, phyAddr, reg, val, hPhy->mdioArgs);
status = hMdio->readC45(phyGroup, phyAddr, mmd, reg, val, hPhy->mdioArgs);
\endcode

\code
// Writing a register using Clause 45
int32_t status;
//status = hMdio->writeC22(phyGroup, phyAddr, reg, val, hPhy->mdioArgs);
status = hMdio->writeC45(phyGroup, phyAddr, mmd, reg, val, hPhy->mdioArgs);
\endcode

# Detailed API Reference

## Clause 45 Read Operation

```c
/**
 * \brief Read a PHY register using Clause 45 addressing
 *
 * \param hPhy         PHY driver handle
 * \param mmd          MMD (Management/MDIO Manageable Device) to access (1-31)
 * \param reg          Register address to read (0-65535)
 * \param val          Pointer to store the read value
 *
 * \return ENET_SOK on success, error code otherwise
 */
int32_t EnetPhy_readC45Reg(EnetPhy_Handle hPhy,
                           uint8_t mmd,
                           uint32_t reg,
                           uint16_t *val);
```

## Clause 45 Write Operation

```c
/**
 * \brief Write to a PHY register using Clause 45 addressing
 *
 * \param hPhy         PHY driver handle
 * \param mmd          MMD (Management/MDIO Manageable Device) to access (1-31)
 * \param reg          Register address to write (0-65535)
 * \param val          Value to write to the register
 *
 * \return ENET_SOK on success, error code otherwise
 */
int32_t EnetPhy_writeC45Reg(EnetPhy_Handle hPhy,
                            uint8_t mmd,
                            uint32_t reg,
                            uint16_t val);
```

## Clause 45 Read-Modify-Write Operation

```c
/**
 * \brief Read-modify-write a PHY register using Clause 45 addressing
 *
 * \param hPhy         PHY driver handle
 * \param mmd          MMD (Management/MDIO Manageable Device) to access (1-31)
 * \param reg          Register address to modify (0-65535)
 * \param mask         Bit mask (1 bits indicate which bits to modify)
 * \param val          Value to write to the masked bits
 *
 * \return ENET_SOK on success, error code otherwise
 */
int32_t EnetPhy_rmwC45Reg(EnetPhy_Handle hPhy,
                          uint8_t mmd,
                          uint32_t reg,
                          uint16_t mask,
                          uint16_t val);
```

# Key Parameters Explained

When using the Clause 45 APIs, you'll encounter these important parameters:

1. **EnetPhy_Handle hPhy**:
   - This is the handle to your PHY instance
   - It contains all the context information about your PHY
   - This is typically obtained when you open/initialize your PHY

2. **mdioArgs**:
   - A structure containing MDIO-specific parameters
   - Includes group ID, timeouts, and other MDIO-specific settings
   - This is automatically handled by the EnetPhy middleware

# Current Limitations

1. **Core EnetPhy Operations**:
   - Some fundamental EnetPhy operations still use Clause 22 internally
   - These include link status detection and PHY aliveness checks
   - These operations will continue to work with Clause 45 PHYs that maintain Clause 22 compatibility

2. **MDIO Manual Mode**:
   - Clause 45 is not supported when MDIO is configured in Manual Mode

3. **PHY-Specific Registers**:
   - For accessing vendor-specific registers, refer to your PHY datasheet for the correct MMD values
   - Some PHY features may require specific sequences of register accesses

# Summary

  Enabling Clause 45 MDIO support provides access to the expanded register space in modern PHY devices. The primary changes involve:

  1. Using the appropriate read/write commands (C45 vs C22)
  2. Adding the MMD parameter to specify the device within the PHY
  3. Managing the expanded register address space (16-bit vs 5-bit)

  These changes enable access to advanced PHY features like Energy Efficient Ethernet, precise timing for IEEE 1588, and vendor-specific extended registers that are not accessible throughF the Clause 22 interface.

# Limitations

1. While the hardware supports MDIO Clause 45 PHY register access, the current implementation in the Ethernet driver (enetphy.c) only utilizes Clause 22 protocol for fundamental PHY operations such as link status detection and PHY aliveness checks. To enable full Clause 45 PHY driver support modifications are required in `source/networking/enet/core/src/phy/enetphy.c` to properly implement the extended register addressing capabilities of Clause 45.

2. Clause 45 is not supported on MDIO Manual Mode [ \ref enetmdio_manualmode ]
