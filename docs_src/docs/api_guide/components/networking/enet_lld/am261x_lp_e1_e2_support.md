Software modification needeed to use Rev-E1 and Rev-E2 version of LP-AM261 EVM{#AM261X_LP_E1_E2_SUPPORT}
=====================

[TOC]
\note You may ignore this page if Ethernet CPSW driver (enet-lld) is not used

 Out-of-box networking (CPSW) examples seamlessly support the Rev-A LP-AM261 board version. Modify the following changes in CPSW example Syscfg-GUI to use Rev-E1 and Rev-E2 board

&nbsp;&nbsp;1. Add two EEPROM instances in 'Syscfg‑Gui → TI BOARD DRIVERS → EEPROM'

   &nbsp;&nbsp;1.1. If 'Mac Port 1' is used, create an EEPROM instance named `CONFIG_EEPROM_PORT1` with I²C address `0x52`.  
   &nbsp;&nbsp;1.2. If 'Mac Port 2' is used, create an EEPROM instance named `CONFIG_EEPROM_PORT2` with I²C address `0x53`.
<div>
<img src="am261x_lp_eeprom_port1.png" alt="CONFIG_EEPROM_PORT1 instance" style="display: inline-block; margin-right: 10px; margin-left: 100px;width: 30%;">
<img src="am261x_lp_eeprom_port2.png" alt="CONFIG_EEPROM_PORT2 instance" style="display: inline-block; margin-left: 80px; width: 30%;">
</div>

&nbsp;&nbsp;2. Update the 'Syscfg‑Gui → TI BOARD DRIVERS → ETHPHY' instances for add‑on PHY usage  

   2.1. If '<a href="https://www.ti.com/tool/DP83826-EVM-AM2">DP83826-EVM-AM2</a>' add-on PHY is used  
   &nbsp;&nbsp;2.1.1. Set 'ETHPHY Device' in `CONFIG_ENET_ETHPHY0` to `DP83826` and 'Phy Address' to `3`.  
   &nbsp;&nbsp;2.1.2. Set 'ETHPHY Device' in `CONFIG_ENET_ETHPHY1` to `DP83826` and 'Phy Address' to `1`.  
   &nbsp;&nbsp;2.1.3. Map `CONFIG_ENET_ETHPHY0` to 'MacPort 1' and `CONFIG_ENET_ETHPHY1` to 'MacPort 2'.  
   &nbsp;&nbsp;2.1.4. Configure either 'MII/RMII' interface in 'CPSW pinmux config'.
<div>
<img src="am261x_lp_dp83826_port1.png" alt="CONFIG_ENET_ETHPHY0 instance" style="display: inline-block; margin-right: 10px; margin-left: 100px;width: 30%;">
<img src="am261x_lp_dp83826_port2.png" alt="CONFIG_ENET_ETHPHY1 instance" style="display: inline-block; margin-left: 80px; width: 30%;">
</div>

   2.2. If '<a href="https://www.ti.com/tool/DP83TG720-EVM-AM2">DP83TG720-EVM-AM2</a>' PHY  
   &nbsp;&nbsp;2.2.1. Set 'ETHPHY Device' in `CONFIG_ENET_ETHPHY0` to `DP83TG720` and 'Phy Address' to `8`.  
   &nbsp;&nbsp;2.2.2. Set 'ETHPHY Device' in `CONFIG_ENET_ETHPHY1` to `DP83TG720` and 'Phy Address' to `12`.  
   &nbsp;&nbsp;2.2.3. Map `CONFIG_ENET_ETHPHY0` to 'MacPort 1' and `CONFIG_ENET_ETHPHY1` to 'MacPort 2'.  
   &nbsp;&nbsp;2.2.4. Set 'Link Speed Capability' and 'Link Duplexity Capability' to `ENET_SPEED_1GBIT` and `ENET_DUPLEX_FULL` for both MAC ports in 'MAC Port Config'.  
   &nbsp;&nbsp;2.2.5. Configure 'RGMII' interface in 'CPSW pinmux config'.
<div>
   <img src="am261x_lp_dp83tg720_port1.png" alt="CONFIG_ENET_ETHPHY0 instance" style="display: inline-block; margin-right: 10px; margin-left: 100px;width: 30%;">
<img src="am261x_lp_dp83tg720_port2.png" alt="CONFIG_ENET_ETHPHY1 instance" style="display: inline-block; margin-left: 80px; width: 30%;">
</div>
