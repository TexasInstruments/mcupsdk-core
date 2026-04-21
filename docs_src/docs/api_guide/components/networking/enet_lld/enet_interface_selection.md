# Ethernet interface (RGMII / MII) selection {#enet_interface_selection}

[TOC]

The diagram below depicts the signals for RGMII and MII respectively.

\imageStyle{enet_RGMII_MII_signals.png,width:50%}
\image html enet_RGMII_MII_signals.png Figure: Ethernet RGMII and MII Signalling

- RGMII is the default interface configured in SDK examples. 
- To configure MII interface follow steps captured below:
   - Modify the following changes in CPSW example Syscfg-GUI

&nbsp;&nbsp;1. SOC Pinmux selection in 'Syscfg‑Gui → TI NETWORKING → Enet (CPSW) → CPSW Pinmux Config'

&nbsp;&nbsp;1.1. If 'Mac Port 1' is used, select `MII` from drop down against `RMII(1)/RGMII(1)/MII(1)`. \n
&nbsp;&nbsp;1.2. If 'Mac Port 2' is used, select `MII` from drop down against `RMII(2)/RGMII(2)/MII(2)`.
<div>
<img src="am26xx_pinmux_mii_por1.png" alt="MII_PINMUX_PORT1 instance" style="display: inline-block; margin-right: 10px; margin-left: 100px;width: 30%; border: 2px solid #000000; padding: 2px;">
<img src="am26xx_pinmux_mii_port2.png" alt="MII_PINMUX_PORT2 instance" style="display: inline-block; margin-left: 80px; width: 40%; border: 2px solid #000000; padding: 2px; ">
</div>

&nbsp;&nbsp;2. MII1 Pinmux selection in 'Syscfg‑Gui → TI NETWORKING → Enet (CPSW) → IO Set'

&nbsp;&nbsp;2.1. If 'Mac Port 1' is used, select `MII1` from drop down against `MII1`. \n
&nbsp;&nbsp;2.2. If 'Mac Port 2' is used, select `MII2` from drop down against `MII2`.
<div>
<img src="am26xx_pinmux_ioset_port1.png" alt="MII_PINMUX_PORT1 instance" style="display: inline-block; margin-right: 10px; margin-left: 100px;width: 30%; border: 2px solid #000000; padding: 2px;">
<img src="am26xx_pinmux_ioset_port2.png" alt="MII_PINMUX_PORT2 instance" style="display: inline-block; margin-left: 80px; width: 40%; border: 2px solid #000000; padding: 2px;">
</div>

&nbsp;&nbsp;3. PHY Pinmux selection for "RX_ER" pin in 'Syscfg‑Gui → TI BOARD DRIVERS → ENETPHY (Enet CPSW/ICSS) → Extended Configuration'

\imageStyle{am26xx_phy_rx_er_pinmux_config.png,width:50%}
\image html am26xx_phy_rx_er_pinmux_config.png Figure: Ethernet PHY RX_ER Pin config

[Back To Top](\ref enet_interface_selection)
