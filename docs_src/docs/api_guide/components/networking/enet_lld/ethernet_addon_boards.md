Ethernet Add-on Boards {#ETHERNET_ADDON_BOARDS_TOP}
======================

[TOC]

# Introduction {#ethernet_addon_boards_intro}
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

AM263Px Control Card(TMDSCND263P) board provides the support for Ethernet add-on boards.
1. Automotive Ethernet add-on Board(<a href="https://www.ti.com/tool/DP83TG720-EVM-AM2">DP83TG720</a>) 
2. Industrial Ethernet add-on Board(<a href="https://www.ti.com/tool/DP83826-EVM-AM2">DP83826e</a>)

The diagram below shoes the top view of AM263Px Control Card which shows the Ethernet add-on board connector.
![Ethernet Add-on Board]
 (Ethernet_addon_board.png "Ethernet Add-on Board")
 
Currently the out-of box support is present only for Automotive Ethernet add-on board for the following examples:
1. \ref EXAMPLES_ENET_CPSW_LOOPBACK
2. \ref EXAMPLES_ENET_LAYER2_CPSW
3. \ref EXAMPLES_ENET_LWIP_CPSW

## Important Usage Guidelines

\note Automotive Ethernet PHY(DP83TG720) doesn't support auto negotiation and works only in 1Gbps.

\note In order to support Automotive Ethernet add-on board on AM263Px Control Card, board modifications are needed.
Please refer to the Table 2-16(Ethernet Routing) in User Guide for board modifications.
<a href="https://www.ti.com/lit/ug/spruj86a/spruj86a.pdf">AM263Px Control Card Evaluation Module User's Guide (Rev. A)</a>

In order to run Out-of-Box examples for Automotive Ethernet PHY(DP83TG720), make following changes in the syscfg tool:
 \imageStyle{eth_add_on_syscfg1.png,width:50%}
 \image html eth_add_on_syscfg1.png
 \imageStyle{eth_add_on_syscfg2.png,width:50%}
 \image html eth_add_on_syscfg2.png  **Figure**: Syscfg tool to configure Ethernet Add-on board(Auto PHY)
 
 Make these below changes in the API EnetApp_initLinkArgs to change the speed and duplexity in order to support Automotive Ethernet PHY(DP83TG720) usecases.
 \imageStyle{speed_duplexity.png,width:50%}
 \image html speed_duplexity.png  **Figure**: Change speed and duplexity
