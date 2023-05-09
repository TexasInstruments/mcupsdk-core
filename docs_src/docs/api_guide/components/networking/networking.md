# Networking {#NETWORKING}

[TOC]

\cond SOC_AM64X || SOC_AM243X || SOC_AM263X

## Overview
Advances in automated factories and smart vehicles require advanced networking capabilities, real-time processing and more advanced motor-control topologies. TI Networking solution packaged with the SDK provide a rapidly growing need for Sitara MCU to out space traditional MCUs and deliver processor-like capabilities. Networking is a broad terms used to cover Ethernet (IEEE 802.3), EtherCAT Profinet and other ethernet-like communication protocols used in industrial, automotive and other general use cases.

  \imageStyle{networking_apps.png,width:50%}
  \image html networking_apps.png Figure: Industrial And Automotive Networking Applications

This document covers driver software architecture, Application Programming Interfaces (API), protocol stack usage, **LwIP** ([Light Weight IP stack](https://savannah.nongnu.org/git/?group=lwip)), examples and demos that are packaged in the SDK. The out-of-box Ethernet (CPSW) Examples enables fast on-board and accelerates the development cycle. This document also provides API guide for low level Ethernet, Industrial communication protocols drivers along with Layer-2 (low level) examples to enable advanced users to integrate the custom stack.

**Networking is supported using following two hardware Peripherals:**
- Common Port SWitch (**CPSW**) : CPSW subsystem provides IEEE 802.3 standard Ethernet gigabit speed packet communication for the device and can also be configured as an Ethernet switch. CPSW supports RGMII and RMII Interfaces.

\cond SOC_AM64X || SOC_AM243X
- Programmable Real-Time Unit and Industrial Communication Subsystem - Gigabit (**PRU-ICSSG**) : PRU-ICSSG is firmware programmable and can take on various personalities like Industrial Communication Protocol Switch (for protocols like EtherCAT, Profinet, EtherNet/IP), Ethernet Switch, Ethernet MAC, Industrial Drives, etc. PRU-ICSSG supports RGMII and MII modes.
\endcond

\cond  SOC_AM263X
- Programmable Real-Time Unit and Industrial Communication Subsystem (PRU-ICSS) : PRU-ICSS is firmware programmable and can take on various personalities like Industrial Communication Protocol Switch (for protocols like EtherCAT, Profinet, EtherNet/IP), Ethernet Switch, Ethernet MAC, Industrial Drives, etc. PRU-ICSS supports MII mode.
\endcond

To know more about the hardware peripherals, please refer to datasheet and Technical Reference Manual (TRM) on the product page:
- [AM2431](https://www.ti.com/product/AM2431), [AM2432](https://www.ti.com/product/AM2432), [AM2434](https://www.ti.com/product/AM2434)
- [AM2634](https://www.ti.com/product/AM2634), [AM2634-Q1](https://www.ti.com/product/AM2634-Q1)
- [AM2732](https://www.ti.com/product/AM2732)
- [AM6411](https://www.ti.com/product/AM6411), [AM6412](https://www.ti.com/product/AM6412), [AM6421](https://www.ti.com/product/AM6421), [AM6422](https://www.ti.com/product/AM6422), [AM6441](https://www.ti.com/product/AM6441), [AM6442](https://www.ti.com/product/AM6442)

### Salient Features

  \imageStyle{netowrking_features.png,width:50%}
  \image html netowrking_features.png Figure: Networking Software Features

### Software Components Overview

Below is the software components overview highlighting mainly the components used in the Networking software development.

\cond SOC_AM64X || SOC_AM243X || SOC_AM263X || SOC_AM273X
  \imageStyle{networking_layer_with_mbedtls.png,width:50%}
  \image html networking_layer_with_mbedtls.png Figure: Networking Software Components Overview
\endcond

\cond SOC_AWR294X
  \imageStyle{netowrking_layer.png,width:50%}
  \image html netowrking_layer.png Figure: Networking Software Components Overview
\endcond

### Examples and Demos
You can find out-of-box examples and demos **[here](\ref EXAMPLES_NETWORKING)**.

\cond SOC_AM263X || SOC_AM273X || SOC_AM243X
### Performance
You can find ethernet performance data **[here](\ref enetlld_performance)**.
\endcond

### Driver

There are separate drivers based on the use-case :

##### Enet LLD

Ethernet Low-Level Driver (\ref ENET_LLD) is a driver that aims at providing an unified interface for standard Ethernet MAC and switch using CPSW and PRU-ICSSG Peripherals.
- \subpage ENET_LLD

##### ICSS-EMAC

Industrial Communications Subsystem Ethernet Media Access Controller (\ref ICSS_EMAC) driver provide APIs to transmit and receive packets with a firmware based Ethernet switch that has been implemented on PRU-ICSS cores. It is used for 100M Industrial Ethernet Protocols like EtherNet/IP, Profinet etc.
- \subpage ICSS_EMAC

\cond SOC_AM64X || SOC_AM243X
#### ICSS TimeSync
\subpage ICSS_TIMESYNC is a separate driver based on ICSS-EMAC, which provides APIs for PTP/1588 v2 receiver implementation on PRU-ICSSG.
\endcond

\endcond

\cond SOC_AM273X || SOC_AWR294X
## Overview
Advances in automated factories and smart vehicles require advanced networking capabilities, real-time processing and more advanced motor-control topologies. TI Networking solution packaged with the SDK provide a rapidly growing need for Sitara MCU to outpace traditional MCUs and deliver processor-like capabilities. Networking is a broad terms used to cover Ethernet (IEEE 802.3) communication protocol used in industrial, automotive and other general use cases.

  \imageStyle{networking_apps.png,width:50%}
  \image html networking_apps.png Figure: Industrial And Automotive Networking Applications

This document covers driver software architecture, Application Programming Interfaces (API), protocol stack usage, ** LwIP ** ([Light Weight IP stack](https://savannah.nongnu.org/git/?group=lwip)), examples and demos that are packaged in the SDK. The out-of-box Ethernet (CPSW) Examples enables fast on-board and accelerates the development cycle. This document also provides API guide for low level Ethernet, Industrial communication protocols drivers along with Layer-2 (low level) examples to enable advanced users to integrate the custom stack.

 Networking is supported using Common Port SWitch (**CPSW**) Peripheral. CPSW subsystem provides IEEE 802.3 standard Ethernet gigabit speed packet communication for the device and can also be configured as an Ethernet switch. CPSW supports RGMII and RMII Interfaces.

To know more about the hardware peripherals, please refer to datasheet and Technical Reference Manual (TRM) on the product page:
- [AM2431](https://www.ti.com/product/AM2431), [AM2432](https://www.ti.com/product/AM2432), [AM2434](https://www.ti.com/product/AM2434)
- [AM2634](https://www.ti.com/product/AM2634), [AM2634-Q1](https://www.ti.com/product/AM2634-Q1)
- [AM2732](https://www.ti.com/product/AM2732)
- [AM6411](https://www.ti.com/product/AM6411), [AM6412](https://www.ti.com/product/AM6412), [AM6421](https://www.ti.com/product/AM6421), [AM6422](https://www.ti.com/product/AM6422), [AM6441](https://www.ti.com/product/AM6441), [AM6442](https://www.ti.com/product/AM6442)

### Salient Features

  \imageStyle{netowrking_features.png,width:50%}
  \image html netowrking_features.png Figure: Networking Software Features

### Software Components Overview

Below is the software components overview highlighting mainly the components used in the Networking software development.

\cond SOC_AM64X || SOC_AM243X || SOC_AM263X || SOC_AM273X
  \imageStyle{networking_layer_with_mbedtls.png,width:50%}
  \image html networking_layer_with_mbedtls.png Figure: Networking Software Components Overview
\endcond

\cond SOC_AWR294X
  \imageStyle{netowrking_layer.png,width:50%}
  \image html netowrking_layer.png Figure: Networking Software Components Overview
\endcond

### Examples and Demos
You can find out-of-box examples and demos **[here](\ref EXAMPLES_NETWORKING)**.

\cond SOC_AM263X || SOC_AM273X || SOC_AM243X
### Performance
You can find ethernet performance data **[here](\ref enetlld_performance)**.
\endcond

### Driver

Ethernet Low-Level Driver (\ref ENET_LLD) is a driver that aims at providing an unified interface for the different Ethernet peripherals found in TI SoCs.
- \subpage ENET_LLD
\endcond

