# TI EtherCAT SubDevice Controller Exceptions {#ETHERCAT_SUBDEVICE_CONTROLLER_EXCEPTIONS}

This page lists the exceptions TI's EtherCAT SubDevice Controller implementation when compared with ET1100 ASIC.

## Register Exceptions {#ETHERCAT_SUBDEVICE_CONTROLLER_EXCEPTIONS_REGISTER_EXCEPTIONS}

TI ESC register map is fully compatible with ET1100 ASIC register map except for the registers and register fields documented below.

<table>
<tr>
    <th> **Register(s)**
    <th> **Remarks**
</tr>
<tr>
    <td> Write Register Enable (0x0020)<br/>
         Write Register Protection (0x0021)<br/>
         ESC Write Enable (0x0030)<br/>
         ESC Write Protection (0x0031)
    <td> **Not available in TI ESC**
</tr>
<tr>
    <td> PDI Control (0x0140)<br/>
         ESC Configuration (0x0141)
    <td> PDI side access is an exception. RW access is enabled to allow loading of 0x140 and 0x141 from EEPROM ADDR: 0x0000 during RESET. TI ESC uses EEPROM emulation mode.<br/>
         For more details, see \ref ETHERCAT_SUBDEVICE_EXCEPTIONS_ESI_EEPROM_EMULATION_SUPPORT.
</tr>
<tr>
    <td> PDI configuration (0x0150)<br/>
         Sync/Latch[1:0] PDI Configuration (0x0151)
    <td> NOT loaded from EEPROM ADDR: 0x0001 during RESET. This is read only and only bit field 3 and 7 are valid i.e. SYNC0/1 mapped to AL Event Request register 0x0220.2/0x0220.3.<br/>
         Beckhoff is going to make proposal to ETG to make EEPROM loading of this register [0x0150:0x0153] on RESET as well as PDI side implementation ESC vendor specific.
</tr>
<tr>
    <td> On-chip bus extended configuration (0x0152:0x0153)
    <td> **Not available in TI ESC**
</tr>
<tr>
    <td> RX Error Counter of Port 0 & 1 (0x0301 and 0x303)
    <td>
         <ul>
         <li>During frame (when RX_DV is asserted)<br/>
            <ul>
            <li>TI ESC increments the counter by reading the PHY register to count RX Error (RX_ERR) during frame (when RX_DV is asserted) in MII_CLK units.</li>
            </ul>
         </li>
         <li>Outside frame (when RX_DV is not asserted)<br/>
            <ul>
            <li>TI ESC polls for RX_ERR interrupt, and interpolates clock cycles based on polling duration. Therefore it does not count RX_ERR in MII_CLK units precisely.</li>
            <li>When only one port is connected, the polling interval is ~120 ns.</li>
            <li>When both ports are connected, the polling interval for Port 0 is ~120 ns and the polling interval for Port 1 is ~680 ns.</li>
            </ul>
         </li>
         </ul>
         **NOTE : Address of PHY register which counts RX Errors should be configured in vendor specific 0x0E28 register in TI ESC for the counting to work during frame**
</tr>
<tr>
    <td> ECAT Processing Unit Error Counter(0x030C)
    <td> Physical layer errors are not counted by TI ESC.
</tr>
<tr>
    <td> PDI Error Counter (0x030D)
    <td> **Not available in TI ESC**
</tr>
<tr>
    <td> Watchdog Divider (0x0400:0x0401)
    <td> For TI ESC max value is: <br/>
         <ul>
         <li>196.6 us or 4915 (13-bit wide and NOT 16-bit) because of 333 MHz clock tick on PRU-ICSSG (AM64x/AM243x)</li>
         <li>327.64 us or 8189 (13-bit wide and NOT 16-bit) because of 200 MHz clock tick on PRU-ICSSM (AM263x)</li>
         </ul>
        If one programs > 13 bits, upper 3-bits are lost as firmware shifts 3-bits (25MHz clock to 333/200 MHz clock conversion by multiplying with 13.32/8) before programming to HW register.
</tr>
<tr>
    <td> Status SyncManager 0 : 0x0805.Bits4-7 <br/>
         Status SyncManager 1 : 0x080D.Bits4-7 <br/>
         Status SyncManager 2 : 0x0815.Bits4-7 <br/>
         Status SyncManager 3 : 0x081D.Bits4-7 <br/>
         Status SyncManager 4 : 0x0825.Bits4-7 <br/>
         Status SyncManager 5 : 0x082D.Bits4-7 <br/>
         Status SyncManager 6 : 0x0835.Bits4-7 <br/>
         Status SyncManager 7 : 0x083D.Bits4-7
    <td> Following Bits are NOT implemented
         Bit 4-5: Buffered mode: buffer status (last written buffer)<br/>
         00: 1. buffer<br/>
         01: 2. buffer<br/>
         10: 3. buffer<br/>
         11: (no buffer written)<br/>
         Mailbox mode: reserved<br/>
         Bit 6: Read buffer in use (opened)<br/>
         Bit 7: Write buffer in use (opened)<br/>
</tr>
<tr>
    <td> Activate SyncManager 0 : 0x806.Bits6-7 <br/>
         Activate SyncManager 1 : 0x80E.Bits6-7 <br/>
         Activate SyncManager 2 : 0x816.Bits6-7 <br/>
         Activate SyncManager 3 : 0x81E.Bits6-7 <br/>
         Activate SyncManager 4 : 0x826.Bits6-7 <br/>
         Activate SyncManager 5 : 0x82E.Bits6-7 <br/>
         Activate SyncManager 6 : 0x836.Bits6-7 <br/>
         Activate SyncManager 7 : 0x83E.Bits6-7
    <td> Following Bits are NOT implemented <br/>
         Bit 6: Latch Event ECAT <br/>
         1: Generate Latch event if EtherCAT MainDevice issues a buffer exchange <br/>
         Bit 7: Latch Event PDI <br/>
         1: Generate Latch events if PDI issues a buffer exchange or if PDI accesses buffer start address
</tr>
<tr>
    <td> Start Time Cyclic Operation (0x0990:0x0997)
    <td> System time of next pulse register (0x990:0x993) is not updated instantaneously, resulting in read of incorrect value if read immediately after sync pulse.
</tr>
<tr>
    <td> Latch0 Status (0x09AE).Bit2<br/>
         Latch1 Status (0x09AF).Bit2
    <td> Following Bit is not implemented<br/>
         Bit 2: Latch0/1 pin state
</tr>
<tr>
    <td> EtherCAT Buffer Change Event Time (0x09F0:0x09F3) <br/>
         PDI Buffer Start Event Time (0x09F8:0x09FB) <br/>
         PDI Buffer Change Event Time (0x09FC:0x09FF)
    <td> **Not available in TI ESC**
</tr>
<tr>
    <td> Digital Outputs (0x0F00:0x0F03)
    <td> Digital Outputs are updated at the end of an EtherCAT frame which triggered the Process Data Watchdog in ET1100 with typical SyncManager configuration: a frame containing a write <br/>
         access to at least one of the registers 0x0F00:0x0F03. TI ESC firmware requires SyncManager address to be higher than or equal to 0x1000.
</tr>
</table>

## Known Issues with no plans to fix

<table>
<tr>
    <th> **Issue**
    <th> **Description**
</tr>
<tr>
    <td> PINDSW-47/SDOCM00092510: Single datagram accessing multiple FMMU mapped areas using LRD/LWR commands from a single SubDevice
    <td> Increased codememory requirements needed in firmware to implement this support. LRW command supports this which is more optimal with lower framing overhead. Minor use case impact as more optimal solutions exists. May cause interop issues with certain MainDevices if 8 SM is supported by SubDevice and all of them are accessed via single logical datagram
</tr>
<tr>
    <td> PINDSW-72/SDOCM00098105: PDI/PD watchdog counter incremented by 1 whenever PDI/PD watchdog is disabled using EtherCAT MainDevice
    <td> Whenever EtherCAT MainDevice disables WD by writing zero to respective Watchdog Time registers (0x0410:0x0411 or 0x0420:0x0421). ICSS h/w limitation, can potentially workaround in firmware by maintaining this counter in firmware but require additional instructions. This has very minor use case impact to undertake this.
</tr>
<tr>
    <td> PINDSW-74/SDOCM00098950: LRD access on unused registers results in WKC increment
    <td> Firmware does not support register protection in LRD mode at this moment, it requires more firmware footprint to support, this minor spec compliance does not justify the footprint increase and there are no Write Only registers in ESC. LRD access to unused register is not a practical use case.
</tr>
<tr>
    <td> PINDSW-141/SDOCM00105048: LRW access to non-interleaved input and output process data of multiple SubDevices does not work
    <td> **Conditions in which failures occur**<br/>
         Single LRW datagram accessing FMMU mapped areas in multiple SubDevices and PD out is mapped.<br/>
         FMMU0(0x1000:0x1007)-> SM2#1(Write SM)<br/>
         FMMU1(0x1008:0x100F)->SM 2#2(Write SM)<br/>
         FMMU2(0x1010:0x1017)->SM 3#1(Read SM)<br/>
         FMMU3(0x1018:0x101F)->SM 3#2(Read SM)<br/>
         Single LRW access from (0x1000:101F)<br/>

         **Root Cause**<br/>
         Pointer management is optimized for interleaved access as well as non-interleaved access I/O data is not a very optimal use of EtherCAT. It increases the cycle time overhead/datagram size and not effective use of LRW datagram which can perform read and write in the same cycle.<br/>

        **Work-around**<br/>
        Use LRD/LWR datagram to access process data Use LRW datagram to access process data Input and output overlaid on the same logical address range (TwinCAT usage)<br/>
        Input and output of a given SubDevice back to back in logical address space<br/>
        FMMU0(0x1000:0x1007)->SM 2 #1(Write SM)<br/>
        FMMU1(0x1008:0x100F)->SM 3#1 (Read SM)<br/>
        FMMU2 (0x1010:0x1017)->SM2 #2 (Write SM)<br/>
        FMMU3(0x1018:0x101F)->SM 3#2(Read SM)<br/>
</tr>
</table>

## Known Functional Differences

<table>
<tr>
    <th> **Functional Difference**
    <th> **Description**
</tr>
<tr>
    <td> Increased Process Path latency
    <td> There are certain scenarios under which TI ESC requires increased process path latency. Please refer to section 3.6.2 of [PRU-ICSS EtherCAT SubDevice Troubleshooting Guide](https://www.ti.com/lit/an/spracj7/spracj7.pdf) for more details.
</tr>
<tr>
    <td> Enhanced link detection using RX_ERR and PHY does not support fast link detection
    <td> RX_ERR detection is valid only during inside frame. This may not be as reliable as ET1100 which supports RX_ERR detection outside frame.
</tr>
<tr>
    <td> APRW/FPRW/BRW for SM mapped process data memory
    <td> This is not a valid use as RW access to SM mapped area does not make sense.
</tr>
<tr>
    <td> LINKACT LED is controlled using the Link/Activity signals from the PHY
    <td> It is recommended to drive the LINKACT LED from ESC, instead of the Link/Activity LED signals of the PHY. <br/>
         **Possible Solution** <br/>
         PHY LEDs can be controlled from R5F by periodically monitoring change in following registers:<br/>
         RX Port0 frame counter(0x0E00:0x0E03)<br/>
         RX Port1 frame counter(0x0E04:0x0E07)<br/>
</tr>
</table>

## ESI EEPROM Emulation Support {#ETHERCAT_SUBDEVICE_EXCEPTIONS_ESI_EEPROM_EMULATION_SUPPORT}

TI ESC follows section "11.2.4 EEPROM Emulation" in [EtherCAT ESC Datasheet Section 1 - Technology](https://download.beckhoff.com/download/Document/io/ethercat-development-products/ethercat_esc_datasheet_sec1_technology_2i3.pdf) from functional principle point of view. This is typically used in IP core based ESCs with non-volatile memory. Here, host CPU emulates all EEPROM operations (read/write/reload) using RAM and from EtherCAT MainDevice point of view, this is equivalent to I2C EEPROM managed by ESC.

In ET1100 ASIC, the EtherCAT MainDevice can invoke reloading the EEPROM content. In this case the Configured Station Alias 0x0012:0x0013 and PDI Control Bit 0x0140.9 (enhanced link detection) are not taken over, they are only taken over at the initial EEPROM loading after power-on or reset.

In IP core based ESCs, however the read data for a reload command (or the initial EEPROM loading) is reduced to the Configured Station Alias (0x0012:0x0013) and the Enhanced Link Detection Enables (0x0140[9], 0x0140[15:12]). IP core uses scheme described in section "2.45.1 EEPROM emulation with 32 bit EEPROM data register" of [EtherCAT ESC Datasheet Section 2 - Register Description](https://download.beckhoff.com/download/Document/io/ethercat-development-products/ethercat_esc_datasheet_sec2_registers_3i0.pdf). There are some exceptions to how this implemented in TI ESC, compared to IP core.

- Configured Station Alias, Enhanced Link Detection Enables and DL status Bit2 is updated directly by FWHAL during initial EEPROM loading operation. This is similar to ET1100 ASIC, and exception to IP core based ESCs.

- PDI configuration (0x150:0x153) are not loaded during EEPROM reload/inital load. However this is programmable if customers want to make it similar to ET1100 ASIC. We kept this way because PDI configuration programmability is limited for TI ESC. This is similar to IP core based ESCs.

- Register Pulse Length of SyncSignals (0x0982:0x983) is updated from EEPROM ADR 0x0002 during reload operation.  This is similar to ET1100 ASIC, and exception to IP core based ESCs.
