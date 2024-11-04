Ethernet Add-on Boards {#ETHERNET_ADDON_BOARDS_TOP}
======================

[TOC]

# Introduction {#ethernet_addon_boards_intro}
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
\cond SOC_AM263PX
AM263Px Control Card(TMDSCND263P) board provides the support for Ethernet add-on boards.
\endcond

\cond SOC_AM261X
LP-AM261 board provides the support for the following Ethernet add-on boards.
\endcond

1. Automotive Ethernet add-on Board(<a href="https://www.ti.com/tool/DP83TG720-EVM-AM2">DP83TG720-EVM-AM2</a>) 
2. Industrial Ethernet add-on Board(<a href="https://www.ti.com/tool/DP83826-EVM-AM2">DP83826-EVM-AM2</a>)

\cond SOC_AM263PX
The diagram below shoes the top view of AM263Px Control Card which shows the Ethernet add-on board connector.
![Ethernet Add-on Board]
 (Ethernet_addon_board.png "Ethernet Add-on Board")
\endcond

\cond SOC_AM261X 
Currently the out-of box support for both the PHYs is present in Ethernet add-on board for the following networking examples:
\ref EXAMPLES_NETWORKING
\endcond

## Important Usage Guidelines

\note Automotive Ethernet PHY(DP83TG720-EVM-AM2) doesn't support auto negotiation and works only in 1Gbps.

\cond SOC_AM261X 
\note Industrial Ethernet PHY(DP83826-EVM-AM2) support has been tested for RMII 100Mbps.
\endcond

\cond SOC_AM263PX 
\note In order to support Automotive Ethernet add-on board on AM263Px Control Card, board modifications are needed.
Please refer to the Table 2-16(Ethernet Routing) in User Guide for board modifications.
<a href="https://www.ti.com/lit/ug/spruj86a/spruj86a.pdf">AM263Px Control Card Evaluation Module User's Guide (Rev. A)</a>

In order to run Out-of-Box examples for Automotive Ethernet PHY(DP83TG720-EVM-AM2), make following changes in the syscfg tool:
 \imageStyle{eth_add_on_syscfg1.png,width:50%}
 \image html eth_add_on_syscfg1.png
 \imageStyle{eth_add_on_syscfg2.png,width:50%}
 \image html eth_add_on_syscfg2.png  **Figure**: Syscfg tool to configure Ethernet Add-on board(Auto PHY)
 
 Make these below changes in the API EnetApp_initLinkArgs to change the speed and duplexity in order to support Automotive Ethernet PHY(DP83TG720-EVM-AM2) usecases.
 \imageStyle{speed_duplexity.png,width:50%}
 \image html speed_duplexity.png  **Figure**: Change speed and duplexity
 \endcond