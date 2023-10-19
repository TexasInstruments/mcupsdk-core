# PRU_ICSSG0 USAGE GUIDE {#ICSSG0_USAGE_GUIDE}

[TOC]

# Introduction

@VAR_SOC_NAME is an extension of Sitara’s industrial-grade portfolio into high-performance microcontrollers. The @VAR_SOC_NAME device is built for industrial applications, such as motor drives and remote I/O modules, which require a combination of real-time communications and processing.
The @VAR_SOC_NAME family provides scalable performance with up to two instances of Sitara’s gigabit TSN-enabled PRU_ICSSG. For more information, see Programmable Real-Time Unit Subsystem and Industrial Communication Subsystem - Gigabit (PRU_ICSSG) section in Processors and Accelerators chapter in the device TRM.

This is a guide to use PRU_ICSSG0 instance in PRU_ICSSG peripheral. On EVMs supported by TI (AM64X-EVM, AM243X-EVM), it is not possible use this, as the PRG0_RGMII1 and PRG0_RGMII0 pins are not connected to Ethernet PHYs on those EVMs. Would need a dedicated custom board to use this PRU_ICSSG0 instance of ICSSG peripheral. PRU_ICSSG1 instance of the ICSSG peripheral has no such limitations.

This is the setup guide to bring up PRU_ICSSG0 instance on your custom board or ICSSG0 enabled device.

\cond SOC_AM64X || SOC_AM243X

On @VAR_SOC_NAME, we do not have out of the box support for ICSSG0. 

  - The PRU_ICSSG0 supported on @VAR_SOC_NAME SoC, but the standard issue TI EVMs and LaunchPads do not have board support.
  - You can have PRU_ICSSG0 support with a custom board. Contact TI for more details and possibilities.

  \imageStyle{soc_details_am64_am243.png,width:30%}
  \image html soc_details_am64_am243.png SoC Architecture and details.

# Platforms Supporting PRU_ICSSG0

The driver is compatible for PRU_ICSSG0 and provides full support out of the box.

 Parameter         | Value
 ------------------|--------------------
 CPU + OS          | r5fss0-0_freertos, r5fss0-0_nortos
 Toolchain         | ti-arm-clang
 SoC               | @VAR_SOC_NAME
 Supported Board   | Custom Board only*


# Configurations supported by PRU_ICSSG0

PRU_ICSSG0 supports all modes and combinations supported by ENET-ICSSG driver. The follwing combinations are tested and verified.

  Parameter          | Value
 --------------------|----------------------
 CPU + OS            | r5fss0-0_freertos, r5fss0-0_nortos
 SoC                 | @VAR_SOC_NAME
 MDIO modes          | Normal mode, Manual
 MII modes           | MII, RGMII
 MAC Modes           | Dual MAC, Switch
 LwIP compatible     | YES

\endcond

# Configuring PRU_ICSSG0 instance

- In Sysconfig, in the ENET (ICSS) module under TI Networking section, set the instance option to select the ICSSG0.
  
  \imageStyle{icssg0_system_config.png,width:30%}
  \image html icssg0_system_config.png ICSSG0 System Configuration.
  
- In the same module mentioned above, set the MII/RGMII option which the custom board's PHY supports.
  
  \imageStyle{icssg0_mii_config.png,width:30%}
  \image html icssg0_mii_config.png ICSSG0 System Configuration.
  
- In the same module, find the EMAC Mode, and select either Dual MAC or Switch operation based on application.
- In Sysconfig, in the ENET (ICSS) module under TI Networking section, in the Board config drop down options, 
  find the custom board checkbox option. Please make sure to have the custom board option ENABLED.
- In the same module mentioned above, in Board config, Update the PHY addresses of the PHYs with on board PHYs of custom board.
  
  \imageStyle{icssg0_board_config.png,width:30%}
  \image html icssg0_board_config.png ICSSG0 System Configuration.
  
- Follow the sequence and use appropriate PHY drivers to setup the PHYs on the add-on card for proper start up.
- Match the configuration options in the application to update with appropriate MII mode and board ID for proper link up.

\note Use an ICSSG0 supporting platform to get this functionality. Refer to Supporting configurations for details.


# See Also

\ref NETWORKING
