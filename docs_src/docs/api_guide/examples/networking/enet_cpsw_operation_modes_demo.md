# Enet CPSW Operation Modes Demo {#ENET_LWIP_CPSW_OPERATING_MODES}

[TOC]

# Introduction

- CPSW operates in two Modes
 - Switch Mode - packets forwarding in enabled and packets are forwarded to Host + Other MAC ports. This is the default configuration for CPSW.
 - MAC mode - packets are only given to Host port and is not forwarded to Other MAC Ports.

\cond SOC_AM64X || SOC_AM243X || SOC_AM263X

## CPSW DUAL MAC Mode
- In this mode, both the MAC ports of @VAR_SOC_NAME are confifured to be in MAC mode.
- This mode requires two Network interfaces to operate.
- It is recommended to use two separate subnets for working with DUAL MAC.
- A single implementation of network interface and shared pool is used for both the Netifs
- Dual MAC feature is illustrated by below image: 

  \imageStyle{dual_mac_operation.png,width:35%}
  \image html dual_mac_operation.png DUAL MAC Operation

\endcond


# See Also

\ref NETWORKING
