# TI EtherCAT SubDevice Controller Register List {#ETHERCAT_SUBDEVICE_CONTROLLER_REGISTER_LIST}

TI ESC is fully register compatible with ET1100 ASIC, for any exceptions to this please search for "Not available in TI ESC" and "PDI access is exception here for TI ESC" in below table. Please see \ref ETHERCAT_SUBDEVICE_CONTROLLER_EXCEPTIONS for additional details.

<table>
<tr>
    <th> **Register name**
    <th> **Register offset**
    <th> **Bit/s**
    <th>**Permissions-ECAT**
    <th>**Permissions-PDI**
    <th> **Reset value**
    <th> **Description**
</tr>
<tr>
    <td> Type
    <td> 0x0000
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-7
    <td> R/-
    <td> R/-
    <td> 0x90
    <td> Type of SubDevice controller
</tr>
<tr>
    <td> Revision
    <td> 0x0001
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-7
    <td> R/-
    <td> R/-
    <td> 5/6
    <td> AM263x : 5 <br/>
         AM64x, AM243x : 6 <br/>
</tr>
<tr>
    <td> Build
    <td> 0x0002-0x0003
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-15
    <td> R/-
    <td> R/-
    <td> 0x50C
    <td> Firmware build version <br/>
</tr>
<tr>
    <td> FMMUs supported
    <td> 0x0004
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-7
    <td> R/-
    <td> R/-
    <td> 8
    <td> Number of FMMUs supported.
</tr>
<tr>
    <td> SyncManagers supported
    <td> 0x0005
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-7
    <td> R/-
    <td> R/-
    <td> 8
    <td> Number of SMs supported.
</tr>
<tr>
    <td> RAM Size
    <td> 0x0006
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-7
    <td> R/-
    <td> R/-
    <td> 59
    <td> Size of Process Data RAM (in KB)
</tr>
<tr>
    <td> Port Descriptor
    <td> 0x0007
    <td>
    <td>
    <td>
    <td> 0x0f
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-1
    <td> R/-
    <td> R/-
    <td> 11b
    <td> Port 0 description.<br/>
    00 - not implemented<br/>
    01 - not configured (SII EEPROM)<br/>
    10 - EBUS<br/>
    11 - MII / RMII
</tr>
<tr>
    <td>
    <td>
    <td> 2-3
    <td> R/-
    <td> R/-
    <td> 11b
    <td> Port 1 description
</tr>
<tr>
    <td>
    <td>
    <td> 4-5
    <td> R/-
    <td> R/-
    <td> 00b
    <td> Port 2 description - **Not available in TI ESC**
</tr>
<tr>
    <td>
    <td>
    <td> 6-7
    <td> R/-
    <td> R/-
    <td> 00b
    <td> Port 3 description - **Not available in TI ESC**
</tr>
<tr>
    <td> ESC Features supported
    <td> 0x0008-0x0009
    <td>
    <td>
    <td>
    <td> 0x008c
    <td> Features supported
</tr>
<tr>
    <td>
    <td>
    <td> 0
    <td> R/-
    <td> R/-
    <td> 0b
    <td> FMMU Operation. 0: byte oriented, 1: bit oriented.<br/>
         TI-ESC supports read-only bit oriented FMMUs (LRD command)
</tr>
<tr>
    <td>
    <td>
    <td> 1
    <td> R/-
    <td> R/-
    <td>
    <td> Reserved. Reads 0
</tr>
<tr>
    <td>
    <td>
    <td> 2
    <td> R/-
    <td> R/-
    <td> 1b
    <td> Distributed clocks - 0: not available 1: available.
</tr>
<tr>
    <td>
    <td>
    <td> 3
    <td> R/-
    <td> R/-
    <td> 1b
    <td> Distributed clocks - 0: 32-bit 1: 64-bit
</tr>
<tr>
    <td>
    <td>
    <td> 4
    <td> R/-
    <td> R/-
    <td> 0b
    <td> Low jitter EBUS:<br/>
         0 - not available (standard jitter)<br/>
         1 - available (jitter minimised)
</tr>
<tr>
    <td>
    <td>
    <td> 5
    <td> R/-
    <td> R/-
    <td> 0b
    <td> Enhanced link detection EBUS:<br/>
         0 - not available<br/>
         1 - available
</tr>
<tr>
    <td>
    <td>
    <td> 6
    <td> R/-
    <td> R/-
    <td> 0b
    <td> Enhanced link detection MII <br/>
         0 - not available <br/>
         1 - available
</tr>
<tr>
    <td>
    <td>
    <td> 7
    <td> R/-
    <td> R/-
    <td> 1b
    <td> Separate handling of FCS errors <br/>
         0 - not supported <br/>
         1 - supported: frames with wrong FCS and additional nibble will be counted separately in Forwarded RX Error Counter
</tr>
<tr>
    <td>
    <td>
    <td> 8
    <td> R/-
    <td> R/-
    <td> 0b
    <td> Enhanced DC SYNC Activation <br/>
         0 - not available <br/>
         1 - available
</tr>
<tr>
    <td>
    <td>
    <td> 9
    <td> R/-
    <td> R/-
    <td> 0b
    <td> LRW Command support <br/>
         0 - supported <br/>
         1 - not supported
</tr>
<tr>
    <td>
    <td>
    <td> 10
    <td> R/-
    <td> R/-
    <td> 0b
    <td> BRW/APRW/FPRW Command support <br/>
         0 - supported <br/>
         1 - not supported
</tr>
<tr>
    <td>
    <td>
    <td> 11
    <td> R/-
    <td> R/-
    <td> 0b
    <td> Fixed SM/FMMU configuration <br/>
         0 - variable configuration <br/>
         1 - fixed configuration
</tr>
<tr>
    <td>
    <td>
    <td> 12-15
    <td> R/-
    <td> R/-
    <td>
    <td> Reserved
</tr>
<tr>
    <td> Configured Station Address
    <td> 0x0010-0x0011
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-15
    <td> R/W
    <td> R/-
    <td> 0
    <td> Node address: used in FPRD/FPWR/FPRW commands
</tr>
<tr>
    <td> Configured Station Alias
    <td> 0x0012-0x0013
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-15
    <td> R/-
    <td> R/W
    <td> 0
    <td> Alias addressed used in FPxx commands. Reset to EEPROM <br/>
         0x0004. Activated by setting bit 0x100:24 / 0x103:0
</tr>
<tr>
    <td>
    <td> 0x0020-0x0021
    <td>
    <td>
    <td>
    <td>
    <td> **Not available in TI ESC**
</tr>
<tr>
    <td>
    <td> 0x0030-0x0031
    <td>
    <td>
    <td>
    <td>
    <td> **Not available in TI ESC**
</tr>
<tr>
    <td>
    <td> 0x0040-0x0041
    <td>
    <td>
    <td>
    <td>
    <td> **Not available in TI ESC**
</tr>
<tr>
    <td> ESC DL Control
    <td> 0x0100-0x0103
    <td>
    <td>
    <td>
    <td> 1
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0
    <td> R/W
    <td> R/-
    <td> 1b
    <td> 0: Non-ECAT packets are forwarded <br/>
         1: non-ECAT packets are destroyed
</tr>
<tr>
    <td>
    <td>
    <td> 1
    <td> R/W
    <td> R/-
    <td> 0b
    <td> 0: Settings written in 0x101 are for permanent use <br/>
         1: New 0x101 settings are used for 1 s, then reverted
</tr>
<tr>
    <td>
    <td>
    <td> 2-7
    <td> R/-
    <td> R/-
    <td> 0
    <td> Reserved
</tr>
<tr>
    <td>
    <td>
    <td> 8-9
    <td> R/W
    <td> R/-
    <td> 00b
    <td> Loop port 0 <br/>
         00 auto <br/>
         01 auto-close / manual open <br/>
         10 always open <br/>
         11 always closed
</tr>
<tr>
    <td>
    <td>
    <td> 10-11
    <td> R/W
    <td> R/-
    <td> 00b
    <td> Loop port 1
</tr>
<tr>
    <td>
    <td>
    <td> 12-13
    <td> R/W
    <td> R/-
    <td> 00b
    <td> Loop port 2 - **Not available in TI ESC**
</tr>
<tr>
    <td>
    <td>
    <td> 14-15
    <td> R/W
    <td> R/-
    <td> 00b
    <td> Loop port 3 - **Not available in TI ESC**
</tr>
<tr>
    <td>
    <td>
    <td> 16-23
    <td>
    <td>
    <td>
    <td> **Not available in TI ESC**
</tr>
<tr>
    <td>
    <td>
    <td> 24
    <td> R/W
    <td> R/-
    <td> 0b
    <td>  0: Ignore station alias, 1: alias used for FPxx
</tr>
<tr>
    <td>
    <td>
    <td> 25-31
    <td> R/-
    <td> R/-
    <td>
    <td> Reserved
</tr>
<tr>
    <td> Physical Read/Write Offset
    <td> 0x0108-0x0109
    <td>
    <td>
    <td>
    <td> 0
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-15
    <td> R/W
    <td> R/-
    <td> 0
    <td> Offset between Read and Write address for R/W Commands (FPRW, APRW). <br/>
         RD_ADR = ADR, WR_ADR = ADR + Offset
</tr>
<tr>
    <td> ESC DL Status
    <td> 0x0110-0x0111
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0
    <td> R/-
    <td> R/-
    <td>
    <td> 0: EEPROM not loaded, PDI not operational (no access to PD RAM) <br/>
         1: EEPROM loaded correctly, PDI operational (access to PD RAM)
</tr>
<tr>
    <td>
    <td>
    <td> 1
    <td> R/-
    <td> R/-
    <td>
    <td> 0: PDI watchdog expired <br/>
         1: PDI watchdog reloaded
</tr>
<tr>
    <td>
    <td>
    <td> 2
    <td> R/-
    <td> R/-
    <td>
    <td> Enhanced link detection <br/>
         0: deactivated on all ports <br/>
         1: activated on at least one port
</tr>
<tr>
    <td>
    <td>
    <td> 3
    <td> R/-
    <td> R/-
    <td>
    <td> Reserved
</tr>
<tr>
    <td>
    <td>
    <td> 4
    <td> R/-
    <td> R/-
    <td>
    <td> Physical link on port 0 <br/>
         0: no link <br/>
         1: link detected
</tr>
<tr>
    <td>
    <td>
    <td> 5
    <td> R/-
    <td> R/-
    <td>
    <td> Physical link on port 1
</tr>
<tr>
    <td>
    <td>
    <td> 6
    <td> R/-
    <td> R/-
    <td>
    <td> Physical link on port 2 - **Not available in TI ESC**
</tr>
<tr>
    <td>
    <td>
    <td> 7
    <td> R/-
    <td> R/-
    <td>
    <td> Physical link on port 3 - **Not available in TI ESC**
</tr>
<tr>
    <td>
    <td>
    <td> 8
    <td> R/-
    <td> R/-
    <td>
    <td> Loop port 0 <br/>
         0: opened <br/>
         1: closed
</tr>
<tr>
    <td>
    <td>
    <td> 9
    <td> R/-
    <td> R/-
    <td>
    <td> Communication on port 0 <br/>
         0: no stable communication <br/>
         1: communication established
</tr>
<tr>
    <td>
    <td>
    <td> 10
    <td> R/-
    <td> R/-
    <td>
    <td> Loop port 1
</tr>
<tr>
    <td>
    <td>
    <td> 11
    <td> R/-
    <td> R/-
    <td>
    <td> Communication on port 1
</tr>
<tr>
    <td>
    <td>
    <td> 12
    <td> R/-
    <td> R/-
    <td>
    <td> Loop port 2 - **Not available in TI ESC**
</tr>
<tr>
    <td>
    <td>
    <td> 13
    <td> R/-
    <td> R/-
    <td>
    <td> Communication on port 2 - **Not available in TI ESC**
</tr>
<tr>
    <td>
    <td>
    <td> 14
    <td> R/-
    <td> R/-
    <td>
    <td> Loop port 3  - **Not available in TI ESC**
</tr>
<tr>
    <td>
    <td>
    <td> 15
    <td> R/-
    <td> R/-
    <td>
    <td> Communication on port 3  - **Not available in TI ESC**
</tr>
<tr>
    <td> AL Control
    <td> 0x120-0x121
    <td>
    <td>
    <td>
    <td> 0
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-3
    <td> R/W
    <td> R/-
    <td>
    <td> 0001: Request INIT state <br/>
         0011: Request BOOTSTRAP state <br/>
         0010: Request PREOP state <br/>
         0100: Request SAFEOP state <br/>
         1000: Request OP state
</tr>
<tr>
    <td>
    <td>
    <td> 4
    <td> R/W
    <td> R/-
    <td>
    <td> 0: no acknowledge of error in 0x130 <br/>
         1: acknowledge error ind in 0x130
</tr>
<tr>
    <td>
    <td>
    <td> 5-15
    <td> R/-
    <td> R/-
    <td>
    <td> Reserved
</tr>
<tr>
    <td> AL Status
    <td> 0x0130-0x0131
    <td>
    <td>
    <td>
    <td> 0
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-3
    <td> R/-
    <td> R/W
    <td>
    <td> Current state - 1: INIT, 3: BOOTSTRAP, 2: PREOP <br/>
         4: SAFEOP, 8: OP
</tr>
<tr>
    <td>
    <td>
    <td> 4
    <td> R/-
    <td> R/W
    <td>
    <td> 0: Device in requested state or bit cleared by command <br/>
         1: Device has not entered requested state or changed state <br/>
         as a result of a local action
</tr>
<tr>
    <td>
    <td>
    <td> 5-15
    <td> R/-
    <td> R/-
    <td>
    <td> Reserved
</tr>
<tr>
    <td> AL Status Code
    <td> 0x0134
    <td>
    <td>
    <td>
    <td> 0
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-15
    <td> R/-
    <td> R/W
    <td>
    <td> AL status code
</tr>
<tr>
    <td> PDI Control
    <td> 0x0140
    <td>
    <td>
    <td>
    <td>
    <td> **PDI access is exception here for TI ESC**
</tr>
<tr>
    <td>
    <td>
    <td> 0-7
    <td> R/-
    <td> **RW**-
    <td> 0x80
    <td> PDI type / EEPROM 0x0000 On-chip bus
</tr>
<tr>
    <td> ESC Configuration
    <td> 0x0141
    <td>
    <td>
    <td>
    <td> 0x0c
    <td> **PDI access is exception here for TI ESC**
</tr>
<tr>
    <td>
    <td>
    <td> 0
    <td> R/-
    <td> **RW**-
    <td> 0b
    <td> Control of AL Status <br/>
         0 - AL Status has to be set by PDI <br/>
         1 - AL Status will be set to value written to AL Control register
</tr>
<tr>
    <td>
    <td>
    <td> 1
    <td> R/-
    <td> **RW**-
    <td> 0b
    <td> Enhanced link detection at all ports <br/>
         0 - disabled <br/>
         1 - enabled at all ports
</tr>
<tr>
    <td>
    <td>
    <td> 2
    <td> R/-
    <td> **RW**-
    <td> 1b
    <td> DC Sync Out unit <br/>
         0 - disabled (power saving) <br/>
         1 - enabled
</tr>
<tr>
    <td>
    <td>
    <td> 3
    <td> R/-
    <td> **RW**-
    <td> 1b
    <td> DC Latch In unit
</tr>
<tr>
    <td>
    <td>
    <td> 4
    <td> R/-
    <td> **RW**-
    <td> 0b
    <td> Enhanced link detection, port 0
</tr>
<tr>
    <td>
    <td>
    <td> 5
    <td> R/-
    <td> **RW**-
    <td> 0b
    <td> Enhanced link detection, port 1
</tr>
<tr>
    <td>
    <td>
    <td> 6
    <td> R/-
    <td> **RW**-
    <td>
    <td> Enhanced link detection, port 2 - **Not available in TI ESC**
</tr>
<tr>
    <td>
    <td>
    <td> 7
    <td> R/-
    <td> **RW**-
    <td>
    <td> Enhanced link detection, port 3 - **Not available in TI ESC**
</tr>
<tr>
    <td> PDI Configuration
    <td> 0x0150
    <td>
    <td>
    <td>
    <td> 0xe0
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-4
    <td> R/-
    <td> RW-
    <td> 0
    <td> Reserved
</tr>
<tr>
    <td>
    <td>
    <td> 5-7
    <td> R/-
    <td> RW-
    <td> 111b
    <td> On chip bus <br/>
         111: TI ESC
</tr>
<tr>
    <td> Sync/Latch PDI Configuration
    <td> 0x0151
    <td>
    <td>
    <td>
    <td> 0x00
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-2
    <td> R/-
    <td> R/-
    <td> 00b
    <td> **Not available in TI ESC**
</tr>
<tr>
    <td>
    <td>
    <td> 3
    <td> R/-
    <td> R/-
    <td> 0b
    <td> SYNC0 mapped to AL Event Request register 0x0220.2: <br/>
         0 - Disabled <br/>
         1 - Enabled
</tr>
<tr>
    <td>
    <td>
    <td> 4-6
    <td> R/-
    <td> R/-
    <td> 00b
    <td> **Not available in TI ESC**
</tr>
<tr>
    <td>
    <td>
    <td> 7
    <td> R/-
    <td> R/-
    <td> 0b
    <td> SYNC1 mapped to AL Event Request register 0x0220.3: <br/>
         0 - Disabled <br/>
         1 - Enabled
</tr>
<tr>
    <td> On-chip bus extended configuration
    <td> 0x0152-0x0153
    <td>
    <td>
    <td>
    <td>
    <td> **Not available in TI ESC**
</tr>
<tr>
    <td> ECAT Event Mask
    <td> 0x0200-0x0201
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-15
    <td> R/W
    <td> R/-
    <td> 0
    <td> For each bit <br/>
         0 - Corresponding bit in 0x210-0x211 will not appear in ECAT IRQ (the IRQ bit is filled with 0 instead) <br/>
         1 - Corresponding bit appears in IRQ
</tr>
<tr>
    <td> AL Event Mask
    <td> 0x0204-0x0207
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-31
    <td> R/-
    <td> R/W
    <td> 0
    <td> For each bit <br/>
         0 - Corresponding bit in 0x220-0x223 will not appear in PDI IRQ (the IRQ bit is filled with 0 instead) <br/>
         1 - Corresponding bit appears in IRQ
</tr>
<tr>
    <td> ECAT Event Request
    <td> 0x0210-0x0211
    <td>
    <td>
    <td>
    <td> 0
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0
    <td> R/-
    <td> R/-
    <td>
    <td> 0 - No change on DC latch inputs <br/>
         1 - At least one change on DC latch inputs <br/>
         Cleared by reading DC latch times (0x9AE) from ECAT
</tr>
<tr>
    <td>
    <td>
    <td> 1
    <td> R/-
    <td> R/-
    <td>
    <td> Reserved
</tr>
<tr>
    <td>
    <td>
    <td> 2
    <td> R/-
    <td> R/-
    <td>
    <td> 0 - No change in DL status <br/>
         1 - DL status changed <br/>
         Cleared by reading DL status (0x110) from ECAT
</tr>
<tr>
    <td>
    <td>
    <td> 3
    <td> R/-
    <td> R/-
    <td>
    <td> 0 - No change in AL status <br/>
         1 - AL status changed <br/>
         Cleared by reading AL status (0x130) from ECAT
</tr>
<tr>
    <td>
    <td>
    <td> 4
    <td> R/-
    <td> R/-
    <td>
    <td> SyncManager Channel 0 event: <br/>
         0 - no pending event <br/>
         1 - event pending
</tr>
<tr>
    <td>
    <td>
    <td> 5
    <td> R/-
    <td> R/-
    <td>
    <td> SyncManager Channel 1 event <br/>
         (Cleared reading 0x806)
</tr>
<tr>
    <td>
    <td>
    <td> 6
    <td> R/-
    <td> R/-
    <td>
    <td> SyncManager Channel 2 event
</tr>
<tr>
    <td>
    <td>
    <td> 7
    <td> R/-
    <td> R/-
    <td>
    <td> SyncManager Channel 3 event
</tr>
<tr>
    <td>
    <td>
    <td> 8
    <td> R/-
    <td> R/-
    <td>
    <td> SyncManager Channel 4 event
</tr>
<tr>
    <td>
    <td>
    <td> 9
    <td> R/-
    <td> R/-
    <td>
    <td> SyncManager Channel 5 event
</tr>
<tr>
    <td>
    <td>
    <td> 10
    <td> R/-
    <td> R/-
    <td>
    <td> SyncManager Channel 6 event
</tr>
<tr>
    <td>
    <td>
    <td> 11
    <td> R/-
    <td> R/-
    <td>
    <td> SyncManager Channel 7 event
</tr>
<tr>
    <td>
    <td>
    <td> 12-15
    <td> R/-
    <td> R/-
    <td>
    <td> Reserved
</tr>
<tr>
    <td> AL Event Request
    <td> 0x0220-0x0223
    <td>
    <td>
    <td>
    <td> 0
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0
    <td> R/-
    <td> R/-
    <td>
    <td> 0 - no request for change in AL state (0x120) <br/>
         1 - request for change in AL state
</tr>
<tr>
    <td>
    <td>
    <td> 1
    <td> R/-
    <td> R/-
    <td>
    <td> 0 - no change in DC latch inputs <br/>
         1 - at least one change in DC latch inputs
</tr>
<tr>
    <td>
    <td>
    <td> 2
    <td> R/-
    <td> R/-
    <td>
    <td> State of DC SYNC0 (if register 0x0151.3=1): Bit is cleared by reading SYNC0 status 0x098E from PDI)
</tr>
<tr>
    <td>
    <td>
    <td> 3
    <td> R/-
    <td> R/-
    <td>
    <td> State of DC SYNC1
</tr>
<tr>
    <td>
    <td>
    <td> 4
    <td> R/-
    <td> R/-
    <td>
    <td> SyncManager activation register <br/>
         (SyncManager register offset 0x6) changed: <br/>
         0 - No change in any SyncManager <br/>
         1 - At least one SyncManager changed <br/>
         (Bit is cleared by reading SyncManager Activation registers 0x0806 etc. from PDI)
</tr>
<tr>
    <td>
    <td>
    <td> 5
    <td> R/-
    <td> R/-
    <td>
    <td> 0 - no EEPROM command pending <br/>
         1 - EEPROM command pending
</tr>
<tr>
    <td>
    <td>
    <td> 6
    <td> R/-
    <td> R/-
    <td>
    <td> Watchdog process data <br/>
         0 - has not expired <br/>
         1 - has expired <br/>
         (Bit is cleared by reading Watchdog Status Process Data 0x0440 from PDI)
</tr>
<tr>
    <td>
    <td>
    <td> 7
    <td> R/-
    <td> R/-
    <td>
    <td> Reserved
</tr>
<tr>
    <td>
    <td>
    <td> 8-23
    <td> R/-
    <td> R/-
    <td>
    <td> 0 - No SM (bit number - 8) interrupt pending <br/>
         1 - SM interrupt pending
</tr>
<tr>
    <td>
    <td>
    <td> 24-31
    <td> R/-
    <td> R/-
    <td>
    <td> Reserved
</tr>
<tr>
    <td> RX Error Counter
    <td> 0x0300-0x0307
    <td>
    <td>
    <td>
    <td> 0
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-7
    <td> R/W
    <td> R/-
    <td> 0
    <td> Invalid frame counter port 0. Counting is stopped when 0xFF is reached. Cleared if any of the counters (0x300-0x30b) is written.The invalid frame counters are incremented if there is an error in the frame format (Preamble, SFD – Start of Frame Delimiter, FCS – Checksum, invalid length)
</tr>
<tr>
    <td>
    <td>
    <td> 8-15
    <td> R/W
    <td> R/-
    <td> 0
    <td> RX Error counter port 0. Counting is stopped when 0xFF is reached. Cleared if any of the counters (0x300-0x30b) is written.
</tr>
<tr>
    <td>
    <td>
    <td> 16-23
    <td> R/W
    <td> R/-
    <td> 0
    <td> Invalid frame counter port 1. Counting is stopped when 0xFF is reached. Cleared if any of the counters (0x300-0x30b) is written.The invalid frame counters are incremented if there is an error in the frame format (Preamble, SFD – Start of Frame Delimiter, FCS – Checksum, invalid length)
</tr>
<tr>
    <td>
    <td>
    <td> 24-31
    <td> R/W
    <td> R/-
    <td> 0
    <td> RX Error counter port 1.Counting is stopped when 0xFF is reached. Cleared if any of the counters (0x300-0x30b) is written.
</tr>
<tr>
    <td>
    <td>
    <td> 32-39
    <td> R/-
    <td> R/-
    <td> 0
    <td> Invalid frame counter port 2 - **Not available in TI ESC**
</tr>
<tr>
    <td>
    <td>
    <td> 40-47
    <td> R/-
    <td> R/-
    <td> 0
    <td> RX Error counter port 2 - **Not available in TI ESC**
</tr>
<tr>
    <td>
    <td>
    <td> 48-55
    <td> R/-
    <td> R/-
    <td> 0
    <td> Invalid frame counter port 3 - **Not available in TI ESC**
</tr>
<tr>
    <td>
    <td>
    <td> 56-63
    <td> R/-
    <td> R/-
    <td> 0
    <td> RX Error counter port 3 - **Not available in TI ESC**
</tr>
<tr>
    <td> Forwarded RX Error Counter
    <td> 0x0308-0x030B
    <td>
    <td>
    <td>
    <td> 0
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-7
    <td> R/W
    <td> R/-
    <td> 0
    <td> Forwarded error counter of Port 0 (counting is stopped when 0xFF is reached). Cleared if one of the RX Error counters 0x0300-0x030B is written.
</tr>
<tr>
    <td>
    <td>
    <td> 8-15
    <td> R/W
    <td> R/-
    <td> 0
    <td> Forwarded error counter of Port 1 (counting is stopped when 0xFF is reached). Cleared if one of the RX Error counters 0x0300-0x030B is written.
</tr>
<tr>
    <td>
    <td>
    <td> 16-23
    <td> R/-
    <td> R/-
    <td> 0
    <td> Forwarded error counter of Port 2 - **Not available in TI ESC**
</tr>
<tr>
    <td>
    <td>
    <td> 24-31
    <td> R/-
    <td> R/-
    <td> 0
    <td> Forwarded error counter of Port 3 - **Not available in TI ESC**
</tr>
<tr>
    <td> ECAT Processing Unit Error Counter
    <td>  0x030C
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-7
    <td> R/W
    <td> R/-
    <td> 0
    <td> ECAT Processing Unit error counter (counting is stopped when 0xFF is reached). Counts errors of frames passing the Processing Unit (e.g., FCS is wrong or datagram structure is wrong). Cleared if register is written.
</tr>
<tr>
    <td> PDI Error Counter
    <td> 0x030D
    <td>
    <td>
    <td>
    <td>
    <td> **Not available in TI ESC**
</tr>
<tr>
    <td> Lost Link Counter
    <td> 0x0310-0x0313
    <td>
    <td>
    <td>
    <td> 0
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-7
    <td> R/W
    <td> R/-
    <td> 0
    <td> Lost Link counter of Port 0 (counting is stopped when 0xff is reached). Counts only if port loop is Auto or Auto-Close. Cleared if one of the Lost Link counter registers is written.
</tr>
<tr>
    <td>
    <td>
    <td> 8-15
    <td> R/W
    <td> R/-
    <td> 0
    <td> Lost Link counter of Port 1
</tr>
<tr>
    <td>
    <td>
    <td> 16-23
    <td> R/-
    <td> R/-
    <td> 0
    <td> Lost Link counter of Port 2 - **Not available in TI ESC**
</tr>
<tr>
    <td>
    <td>
    <td> 24-31
    <td> R/-
    <td> R/-
    <td> 0
    <td> Lost Link counter of Port 3 - **Not available in TI ESC**
</tr>
<tr>
    <td> Watchdog Divider
    <td> 0x0400-0x0401
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-12
    <td> R/W
    <td> R/-
    <td> 0x9c2
    <td> (Number of 25 MHz ticks representing basic clock increment for watchdog) - 2. Default value is 0x9c2 = 2498 i.e. 100 us per WD cycle. <br/>
         **NOTE: For TI ESC max value is:** <br/>
         <ul>
         <li>**327.64 us or 8189 (13-bit wide and NOT 16-bit) for 200 MHz clock tick on PRU-ICSSG (AM64x/AM243x) and PRU-ICSSM (AM263x)**</li>
         <li>**196.6 us or 4915 (13-bit wide and NOT 16-bit) for 333 MHz clock tick on PRU-ICSSG (AM64x/AM243x)**</li>
         </ul>
</tr>
<tr>
    <td> Watchdog Time PDI
    <td> 0x0410-0x0411
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-15
    <td> R/W
    <td> R/-
    <td> 0x3e8
    <td> Watchdog time PDI in cycles. Cycle defined by 0x400-0x401. Default value is 1000 for 100ms
</tr>
<tr>
    <td> Watchdog Time Process Data
    <td> 0x0420-0x0421
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-15
    <td> R/W
    <td> R/-
    <td> 0x3e8
    <td> Watchdog time PD in cycles. Default value is 1000 for 100ms
</tr>
<tr>
    <td> Watchdog Status Process Data
    <td> 0x0440-0x0441
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-15
    <td> R/-
    <td> R/-
    <td> 0
    <td> Process Data watchdog <br/>
         0 - Expired <br/>
         1 - Active or Disabled <br/>
         Reading this from PDI clears 0x220.6
</tr>
<tr>
    <td> Watchdog Counter Process Data
    <td> 0x0442
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-7
    <td> R/W
    <td> R/-
    <td> 0
    <td> Write from ECAT clears the register. Number of times PD watchdog expired. Counting stopped at 0xff. (Counter also increases by 1 every time PD watchdog is disabled.)
</tr>
<tr>
    <td> Watchdog Counter PDI
    <td> 0x0443
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-7
    <td> R/W
    <td> R/-
    <td> 0
    <td> Write from ECAT clears the register. Number of times PDI watchdog expired. Counting stopped at 0xff. (Counter also increases by 1 every time PDI watchdog is disabled.)
</tr>
<tr>
    <td> EEPROM Configuration
    <td> 0x0500
    <td>
    <td>
    <td>
    <td> 0
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0
    <td> R/W
    <td> R/-
    <td> 0b
    <td> 0 - EEPROM control is not offerred to PDI <br/>
         1 - EEPROM control is offerred to PDI
</tr>
<tr>
    <td>
    <td>
    <td> 1
    <td> R/W
    <td> R/-
    <td> 0b
    <td> Force EEPROM access for ECAT <br/>
         0 - Do not change bit 0x501.0 <br/>
         1 - Reset 0x501.0 to 0
</tr>
<tr>
    <td>
    <td>
    <td> 2-7
    <td> R/-
    <td> R/-
    <td>
    <td> Reserved
</tr>
<tr>
    <td> EEPROM PDI Access State
    <td> 0x0501
    <td>
    <td>
    <td>
    <td> 0
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0
    <td> R/-
    <td> R/(W)
    <td> 0b
    <td> 0 - PDI releases EEPROM access <br/>
         1 - PDI takes EEPROM access <br/>
         PDI write access possible only when 0x500.0 = 1 and <br/>
         0x500.1 = 0
</tr>
<tr>
    <td> EEPROM Control/Status
    <td> 0x0502-0x0503
    <td>
    <td>
    <td>
    <td> 0x0060
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0
    <td> R/W
    <td> R/-
    <td> 0b
    <td> ECAT write enable - to be set whenever WRITE instruction is given from 0x502.8-10
</tr>
<tr>
    <td>
    <td>
    <td> 1-4
    <td> R/-
    <td> R/-
    <td>
    <td> Reserved
</tr>
<tr>
    <td>
    <td>
    <td> 5
    <td> R/-
    <td> R/-
    <td> 1b
    <td> EEPROM emulation <br/>
         0 - normal operation. I2C is used <br/>
         1 (TI) - PDI emulates EEPROM, I2C is not used
</tr>
<tr>
    <td>
    <td>
    <td> 6
    <td> R/-
    <td> R/-
    <td> 1b
    <td> Supported number of EEPROM bytes: <br/>
         0 - 4 bytes <br/>
         1 - 8 bytes
</tr>
<tr>
    <td>
    <td>
    <td> 7
    <td> R/-
    <td> R/-
    <td> 0b
    <td> EEPROM addressing: <br/>
         0 - 1 address byte (1KBit – 16KBit EEPROMs) <br/>
         1 - 2 address bytes (32KBit – 4 MBit EEPROMs)
</tr>
<tr>
    <td>
    <td>
    <td> 8-10
    <td> R/(W)
    <td> R/(W)
    <td> 000b
    <td> Write access depends on 0x500-0x501 <br/>
         EEPROM command <br/>
         000 - no command (write clears EEPROM error bits) <br/>
         001 - READ (causes 'EEPROM address' (0x504-0x507) to be read and data placed in EEPROM data (0x508-5x50f) <br/>
         010 - WRITE <br/>
         100 - RELOAD. Verify checksum, if valid, data in EEPROM to be written in configuration registers (except Station Alias, which needs reset for reload)
</tr>
<tr>
    <td>
    <td>
    <td> 11
    <td> R/-
    <td> R/-
    <td> 0b
    <td> Checksum error. Set when RELOAD is instructed with incorrect checksum. Reset when another EEPROM command is issued.
</tr>
<tr>
    <td>
    <td>
    <td> 12
    <td> R/-
    <td> R/-
    <td> 0b
    <td> EEPROM loading status <br/>
         0 - completed successfully. EEPROM configuration available in registers. <br/>
         1 - in progress or finished with failure
</tr>
<tr>
    <td>
    <td>
    <td> 13
    <td> R/-
    <td> R/(W)
    <td> 0b
    <td> W available for PDIs emulating EEPROM. <br/>
         0 - Last EEPROM command carried out successfully <br/>
         1 - Invalid EEPROM command or previous command not completed by emulator
</tr>
<tr>
    <td>
    <td>
    <td> 14
    <td> R/-
    <td> R/-
    <td> 0b
    <td> Set when WRITE is issued without 0x502.0 set. Reset when NO COMMAND is issued
</tr>
<tr>
    <td>
    <td>
    <td> 15
    <td> R/-
    <td> R/-
    <td> 0b
    <td> EEPROM interface is busy
</tr>
<tr>
    <td> EEPROM Address
    <td> 0x0504-0x0507
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-32
    <td> R/(W)
    <td> R/(W)
    <td> 0
    <td> Write access depends on 0x500-0x501 EEPROM address on which EEPROM READ or WRITE is to be carried out
</tr>
<tr>
    <td> EEPROM Data
    <td> 0x0508-0x050f
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-64
    <td> R/(W)
    <td> R/(W)
    <td> 0
    <td> Write access depends on 0x500-0x501. Data read from / to write in EEPROM Address (0x504-0x507)
</tr>
<tr>
    <td> MII Management
    <td> 0x0510-0x0511
    <td>
    <td>
    <td>
    <td> 0
    <td> MII Settings
</tr>
<tr>
    <td> Control/Status
    <td>
    <td>
    <td>
    <td>
    <td> 0
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0
    <td> R/W
    <td> R/-
    <td>
    <td> Write enable. Analogous to 0x502.0
</tr>
<tr>
    <td>
    <td>
    <td> 1
    <td> R/W
    <td> R/-
    <td>
    <td> 0: Only ECAT controls MII via these registers <br/>
         1: PDI can also use these registers to modify MII settings. <br/>
         Note: For TI SubDevices, PDI can modify MII settings using FWHAL API. See FWHAL API guide for details.
</tr>
<tr>
    <td>
    <td>
    <td> 2
    <td> R/W
    <td> R/-
    <td>
    <td> MI link detection (link configuration, link detection, registers 0x0518-0x051B): <br/>
         0 - Not available <br/>
         1 - MI link detection active
</tr>
<tr>
    <td>
    <td>
    <td> 3-7
    <td> R/W
    <td> R/-
    <td>
    <td> PHY address offset
</tr>
<tr>
    <td>
    <td>
    <td> 8-9
    <td> R/W
    <td> R/-
    <td>
    <td> Command register: <br/>
         00 - no command <br/>
         01 - read <br/>
         10 - write
</tr>
<tr>
    <td>
    <td>
    <td> 10-12
    <td> R/W
    <td> R/-
    <td>
    <td> Reserved
</tr>
<tr>
    <td>
    <td>
    <td> 13
    <td> R/W
    <td> R/-
    <td>
    <td> Read error: bit set when error during MII read (PHY/register not available). Cleared by writing to this register.
</tr>
<tr>
    <td>
    <td>
    <td> 14
    <td> R/W
    <td> R/-
    <td>
    <td> Command error: <br/>
         0 - last command was successful <br/>
         1 - invalid command or Write without Write enable
</tr>
<tr>
    <td>
    <td>
    <td> 15
    <td> R/W
    <td> R/-
    <td>
    <td> 0 - MII control state machine is idle <br/>
         1 - MII control state machine is busy
</tr>
<tr>
    <td> PHY Address
    <td> 0x0512
    <td>
    <td>
    <td>
    <td> 0
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-4
    <td> R/W
    <td> R/-
    <td> 0
    <td> PHY address: Address PHY whose settings should be modified / read by next MII command
</tr>
<tr>
    <td>
    <td>
    <td> 5-7
    <td> R/W
    <td> R/-
    <td>
    <td> Reserved
</tr>
<tr>
    <td> PHY Register Address
    <td> 0x0513
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-4
    <td> R/W
    <td> R/-
    <td> 0
    <td> Address of the register within the PHY
</tr>
<tr>
    <td>
    <td>
    <td> 5-7
    <td> R/W
    <td> R/-
    <td>
    <td> Reserved
</tr>
<tr>
    <td> PHY Data
    <td> 0x0514-0x0515
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-15
    <td> R/W
    <td> R/-
    <td> 0
    <td> PHY read/write data
</tr>
<tr>
    <td> Logical Start Address FMMU 0
    <td> 0x0600-0x0603
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-31
    <td> R/W
    <td> R/-
    <td> 0
    <td> FMMU 0: Logical address
</tr>
<tr>
    <td> Length FMMU 0
    <td> 0x0604-0x0605
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-15
    <td> R/W
    <td> R/-
    <td> 0
    <td> FMMU 0: Length
</tr>
<tr>
    <td> Logical Start bit FMMU 0
    <td> 0x0606
    <td>
    <td>
    <td>
    <td> 0
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-2
    <td> R/W
    <td> R/-
    <td>
    <td> FMMU 0 start bit. Logical starting bit that shall be mapped (bits are counted from least significant bit (=0) to most significant bit(=7)
</tr>
<tr>
    <td>
    <td>
    <td> 3-7
    <td> R/W
    <td> R/-
    <td>
    <td> Reserved
</tr>
<tr>
    <td> Logical Stop bit FMMU 0
    <td> 0x0607
    <td>
    <td>
    <td>
    <td> 0
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-2
    <td> R/W
    <td> R/-
    <td>
    <td> FMMU 0 ending bit. Last logical bit that shall be mapped (bits are counted from least significant bit (=0) to most significant bit(=7)
</tr>
<tr>
    <td>
    <td>
    <td> 3-7
    <td> R/W
    <td> R/-
    <td>
    <td> Reserved
</tr>
<tr>
    <td> Physical Start Address FMMU 0
    <td> 0x0608-0x0x609
    <td>
    <td>
    <td>
    <td> 0
    <td>
<tr>
    <td>
    <td>
    <td> 0-15
    <td> R/W
    <td> R/-
    <td>
    <td> FMMU 0 physical start address
</tr>
<tr>
    <td> Physical Start bit FMMU 0
    <td> 0x060A
    <td>
    <td>
    <td>
    <td> 0
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-2
    <td> R/W
    <td> R/-
    <td>
    <td> FMMU 0 physical start bit.
</tr>
<tr>
    <td>
    <td>
    <td> 3-7
    <td> R/W
    <td> R/-
    <td>
    <td> Reserved
</tr>
<tr>
    <td> Type FMMU 0
    <td> 0x060B
    <td>
    <td>
    <td>
    <td> 0
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0
    <td> R/W
    <td> R/-
    <td>
    <td> 0 - Ignore FMMU 0 mapping for read accesses <br/>
         1 - Use mapping for read accesses
</tr>
<tr>
    <td>
    <td>
    <td> 1
    <td> R/W
    <td> R/-
    <td>
    <td> 0 - Ignore FMMU 0 mapping for write accesses <br/>
         1 - Use mapping for write accesses
</tr>
<tr>
    <td>
    <td>
    <td> 2-7
    <td> R/W
    <td> R/-
    <td>
    <td> Reserved
</tr>
<tr>
    <td> Activate FMMU 0
    <td> 0x60C
    <td>
    <td>
    <td>
    <td> 0
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0
    <td> R/W
    <td> R/-
    <td>
    <td> Deactivate FMMU 0
</tr>
<tr>
    <td>
    <td>
    <td> 1-7
    <td> R/W
    <td> R/-
    <td>
    <td> Reserved
</tr>
<tr>
    <td> Reserved FMMU 0
    <td> 0x060D-0x060F
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-23
    <td> R/-
    <td> R/-
    <td>
    <td> Reserved
</tr>
<tr>
    <td>
    <td> 0x0610-0x061F
    <td> 0-127
    <td> R/(W)
    <td> R/-
    <td>
    <td> Configuration for FMMU 1
</tr>
<tr>
    <td>
    <td> 0x0620-0x062F
    <td> 0-127
    <td> R/(W)
    <td> R/-
    <td>
    <td> Configuration for FMMU 2
</tr>
<tr>
    <td>
    <td> 0x0630-0x063F
    <td> 0-127
    <td> R/(W)
    <td> R/-
    <td>
    <td> Configuration for FMMU 3
</tr>
<tr>
    <td>
    <td> 0x0640-0x064F
    <td> 0-127
    <td> R/(W)
    <td> R/-
    <td>
    <td> Configuration for FMMU 4
</tr>
<tr>
    <td>
    <td> 0x0650-0x065F
    <td> 0-127
    <td> R/(W)
    <td> R/-
    <td>
    <td> Configuration for FMMU 5
</tr>
<tr>
    <td>
    <td> 0x0660-0x066F
    <td> 0-127
    <td> R/(W)
    <td> R/-
    <td>
    <td> Configuration for FMMU 6
</tr>
<tr>
    <td>
    <td> 0x0670-0x067F
    <td> 0-127
    <td> R/(W)
    <td> R/-
    <td>
    <td> Configuration for FMMU 7
</tr>
<tr>
    <td> Physical Start Address SyncManager 0
    <td> 0x0800-0x0801
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-15
    <td> R/W
    <td> R/-
    <td> 0
    <td> SM 0 starting physical address
</tr>
<tr>
    <td> Length SyncManager 0
    <td> 0x0802-0x0803
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-15
    <td> R/W
    <td> R/-
    <td> 0
    <td> SM 0 Length: SM is disabled if length is equal to zero
</tr>
<tr>
    <td> Control Register SyncManager 0
    <td> 0x0804
    <td>
    <td>
    <td>
    <td> 0
    <td> Note: Write access only when SM 0 is disabled
</tr>
<tr>
    <td>
    <td>
    <td> 0-1
    <td> R/(W)
    <td> R/-
    <td>
    <td> SM 0 mode: <br/>
         00 - 3-buffer <br/>
         10 - 1-buffer (mailbox) <br/>
         01, 11 - reserved
</tr>
<tr>
    <td>
    <td>
    <td> 2-3
    <td> R/(W)
    <td> R/-
    <td>
    <td> SM 0 direction: <br/>
         00 - ECAT read, PDI write <br/>
         01 - ECAT write, PDI read <br/>
         10, 11 - reserved
</tr>
<tr>
    <td>
    <td>
    <td> 4
    <td> R/(W)
    <td> R/-
    <td>
    <td> SM 0 interrupt in ECAT request register (0x210-0x211) <br/>
         0 - no <br/>
         1 - yes
</tr>
<tr>
    <td>
    <td>
    <td> 5
    <td> R/(W)
    <td> R/-
    <td>
    <td> SM 0 interrupt in PDI request register (0x220)
</tr>
<tr>
    <td>
    <td>
    <td> 6
    <td> R/(W)
    <td> R/-
    <td>
    <td> Watchdog trigger <br/>
         0 - disabled <br/>
         1 - enabled
</tr>
<tr>
    <td>
    <td>
    <td> 7
    <td> R/-
    <td> R/-
    <td>
    <td> Reserved
</tr>
<tr>
    <td> Status Register SyncManager 0
    <td> 0x0805
    <td>
    <td>
    <td>
    <td> 0
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0
    <td> R/-
    <td> R/-
    <td>
    <td> Interrupt Write: <br/>
         1: Interrupt after buffer was completely and successfully written <br/>
         0: Interrupt cleared after first byte of buffer was read
</tr>
<tr>
    <td>
    <td>
    <td> 1
    <td> R/-
    <td> R/-
    <td>
    <td> Interrupt Read: <br/>
         1: Interrupt after buffer was completely and successful read <br/>
         0: Interrupt cleared after first byte of buffer was written
</tr>
<tr>
    <td>
    <td>
    <td> 2
    <td> R/-
    <td> R/-
    <td>
    <td> Reserved
</tr>
<tr>
    <td>
    <td>
    <td> 3
    <td> R/-
    <td> R/-
    <td>
    <td> Mailbox mode: mailbox status: <br/>
         0: Mailbox empty <br/>
         1: Mailbox full <br/>
         Buffered mode: reserved
</tr>
<tr>
    <td>
    <td>
    <td> 4-5
    <td> R/-
    <td> R/-
    <td>
    <td> Buffered mode: buffer status (last written buffer): <br/>
         **Not available in TI ESC** <br/>
         00: 1. buffer <br/>
         01: 2. buffer <br/>
         10: 3. buffer <br/>
         11: (no buffer written) <br/>
         Mailbox mode: reserved
</tr>
<tr>
    <td>
    <td>
    <td> 6
    <td> R/-
    <td> R/-
    <td>
    <td> Read buffer in use (opened)  **Not available in TI ESC**
</tr>
<tr>
    <td>
    <td>
    <td> 7
    <td> R/-
    <td> R/-
    <td>
    <td> Write buffer in use (opened)  **Not available in TI ESC**
</tr>
<tr>
    <td> Activate SyncManager 0
    <td> 0x0806
    <td>
    <td>
    <td>
    <td> 0
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0
    <td> R/W
    <td> R/-
    <td>
    <td> SyncManager Enable/Disable: <br/>
         0: Disable: Access to Memory without SyncManager control <br/>
         1: Enable: SyncManager is active and controls Memory area set in configuration
</tr>
<tr>
    <td>
    <td>
    <td> 1
    <td> R/W
    <td> R/-
    <td>
    <td> Repeat Request: <br/>
         A toggle of Repeat Request means that a mailbox retry is needed (primarily used in conjunction with ECAT Read Mailbox)
</tr>
<tr>
    <td>
    <td>
    <td> 2-5
    <td> R/W
    <td> R/-
    <td>
    <td> Reserved
</tr>
<tr>
    <td>
    <td>
    <td> 6
    <td> R/W
    <td> R/-
    <td>
    <td> Latch Event ECAT: **Not available in TI ESC**
</tr>
<tr>
    <td>
    <td>
    <td> 7
    <td> R/W
    <td> R/-
    <td>
    <td> Latch Event PDI: **Not available in TI ESC**
</tr>
<tr>
    <td> PDI Activate SyncManager 0
    <td> 0x0807
    <td>
    <td>
    <td>
    <td> 0
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0
    <td> R/-
    <td> R/W
    <td>
    <td> Deactivate SyncManager: <br/>
         Read: <br/>
         0: Normal operation, SyncManager activated. <br/>
         1: SyncManager deactivated and reset SyncManager locks access to Memory area. <br/>
         Write: <br/>
         0: Activate SyncManager <br/>
         1: Request SyncManager deactivation <br/>
         NOTE: Writing 1 is delayed until the end of a frame which is currently processed.
</tr>
<tr>
    <td>
    <td>
    <td> 1
    <td> R/-
    <td> R/W
    <td>
    <td> Repeat Ack: <br/>
         If this is set to the same value as set by Repeat Request, the PDI acknowledges the execution of a previous set Repeat request.
</tr>
<tr>
    <td>
    <td>
    <td> 2-7
    <td> R/-
    <td> R/W
    <td>
    <td> Reserved
</tr>
<tr>
    <td>
    <td> 0x0808-0x080F
    <td> 0-63
    <td>
    <td>
    <td>
    <td> Configuration for SM 1
</tr>
<tr>
    <td>
    <td> 0x0810-0x0817
    <td> 0-63
    <td>
    <td>
    <td>
    <td> Configuration for SM 2
</tr>
<tr>
    <td>
    <td> 0x0818-0x081F
    <td> 0-63
    <td>
    <td>
    <td>
    <td> Configuration for SM 3
</tr>
<tr>
    <td>
    <td> 0x0820-0x0827
    <td> 0-63
    <td>
    <td>
    <td>
    <td> Configuration for SM 4
</tr>
<tr>
    <td>
    <td> 0x0828-0x082F
    <td> 0-63
    <td>
    <td>
    <td>
    <td> Configuration for SM 5
</tr>
<tr>
    <td>
    <td> 0x0830-0x0837
    <td> 0-63
    <td>
    <td>
    <td>
    <td> Configuration for SM 6
</tr>
<tr>
    <td>
    <td> 0x0838-0x083F
    <td> 0-63
    <td>
    <td>
    <td>
    <td> Configuration for SM 7
</tr>
<tr>
    <td> Receive Time Port 0
    <td> 0x0900-0x0903
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-31
    <td> R/W*
    <td> R/-
    <td> 0
    <td> Receive time port 0 :<br/>
         Write:<br/>
         A write access to register 0x0900 with BWR, APWR (any address) or FPWR (configured address) latches the local time of the beginning of the receive frame (start first bit of preamble) at respective port.<br/>
         Read:<br/>
         Local time of the beginning of the last receive frame containing a write access to this register. NOTE: The time stamps cannot be read in the same frame in which this register was written.
</tr>
<tr>
    <td>
    <td> 0x0904-0x0907
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-31
    <td> R/-
    <td> R/-
    <td> 0
    <td> Receive time port 1
</tr>
<tr>
    <td>
    <td> 0x0908-0x090B
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-31
    <td> R/-
    <td> R/-
    <td> 0
    <td> Receive time port 2 - **Not available in TI ESC**
</tr>
<tr>
    <td>
    <td> 0x090C-0x090F
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-31
    <td> R/-
    <td> R/-
    <td> 0
    <td> Receive time port 3 - **Not available in TI ESC**
</tr>
<tr>
    <td> System Time
    <td> 0x0910-0x0917
    <td>
    <td>
    <td>
    <td> 0
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-63
    <td> R/-
    <td> R/-
    <td>
    <td> ECAT read access: Local copy of the System Time when the frame passed the reference clock (i.e., including System Time Delay). Time latched at beginning of the frame (Ethernet SOF delimiter).<br/>
         PDI read access: Local copy of the System Time. Time latched when reading first byte (0x0910)
</tr>
<tr>
    <td>
    <td>
    <td> 0-31
    <td> W
    <td> R/-
    <td>
    <td> Write access: Written value will be compared with the local copy of the System time. The result is an input to the time control loop.
</tr>
<tr>
    <td> Receive Time ECAT Processing Unit
    <td> 0x0918-0x0919
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-63
    <td> R/-
    <td> R/-
    <td> 0
    <td> Local time of the beginning of a frame (start first bit of preamble) received at the ECAT Processing Unit containing a write access to Register 0x0900. NOTE: Example, if port 0 is open, this register reflects the Receive Time Port 0 as a 64 bit value.
</tr>
<tr>
    <td> System Time Offset
    <td> 0x0920-0x0927
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-63
    <td> R/W
    <td> R/-
    <td> 0
    <td> Difference between local time and System Time. Offset is added to the local time.
</tr>
<tr>
    <td> System Time Delay
    <td> 0x0928-0x092B
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-31
    <td> R/W
    <td> R/-
    <td> 0
    <td> Delay between Reference Clock and the ESC
</tr>
<tr>
    <td> System Time Difference
    <td> 0x092C-0x092F
    <td>
    <td>
    <td>
    <td> 0
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-30
    <td> R/-
    <td> R/-
    <td> 0
    <td> Mean difference between local copy of System Time and received System Time values
</tr>
<tr>
    <td>
    <td>
    <td> 31
    <td> R/-
    <td> R/-
    <td> 0
    <td> 0: Local copy of System Time greater than or equal received System Time<br/>
         1: Local copy of System Time smaller than received System Time
</tr>
<tr>
    <td> Speed Counter Start
    <td> 0x0930-0x0931
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-14
    <td> R/W
    <td> R/-
    <td> 0
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 15
    <td> R/W
    <td> R/-
    <td> Reserved
    <td>
</tr>
<tr>
    <td> Speed Counter Diff
    <td> 0x0932-0x0933
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-15
    <td> R/-
    <td> R/-
    <td>
    <td> Representation of the deviation between local clock period and Reference Clock's clock period (representation: two's complement) Range: ±(Speed Counter Start – 0x7F)
</tr>
<tr>
    <td> System Time Difference Filter Depth
    <td> 0x0934
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-3
    <td> R/W
    <td> R/-
    <td> 0x4
    <td> Filter depth for averaging the received System Time deviation - **TI ESC supports 0, 2, 4, 8, 16 only**. Rounded to next larger power of 2. 0xc selects 16
</tr>
<tr>
    <td>
    <td>
    <td> 4-7
    <td> R/W
    <td> R/-
    <td>
    <td> Reserved
</tr>
<tr>
    <td> Speed Counter Filter Depth
    <td> 0x0935
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-3
    <td> R/W
    <td> R/-
    <td> 0xc
    <td> **TI ESC supports 0, 2, 4, 8, 16 only**. Rounded to next larger power of 2. 0xc selects 16
</tr>
<tr>
    <td>
    <td>
    <td> 4-7
    <td> R/W
    <td> R/-
    <td>
    <td> Reserved
</tr>
<tr>
    <td> Cyclic Unit Control
    <td> 0x0980
    <td>
    <td>
    <td>
    <td> 0
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0
    <td> R/-
    <td> R/-
    <td> 0b
    <td> SYNC out unit control: <br/>
         0: ECAT controlled <br/>
         1: PDI controlled
</tr>
<tr>
    <td>
    <td>
    <td> 1-3
    <td> R/-
    <td> R/-
    <td>
    <td> Reserved
</tr>
<tr>
    <td>
    <td>
    <td> 4
    <td> R/-
    <td> R/-
    <td> 0b
    <td> Latch In unit 0: <br/>
         0: ECAT controlled <br/>
         1: PDI controlled <br/>
         NOTE: Always 1 (PDI controlled) if System Time is PDI controlled. Latch interrupt is routed to ECAT/PDI depending on this setting.
</tr>
<tr>
    <td>
    <td>
    <td> 5
    <td> R/-
    <td> R/-
    <td> 0b
    <td>  Latch In unit 1
</tr>
<tr>
    <td>
    <td>
    <td> 6-7
    <td> R/-
    <td> R/-
    <td>
    <td> Reserved
</tr>
<tr>
    <td> Activation register
    <td> 0x0981
    <td>
    <td>
    <td>
    <td> 0
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0
    <td> R/W
    <td> R/-
    <td> 0b
    <td> Sync Out Unit activation: <br/>
         0: Deactivated <br/>
         1: Activated <br/>
         NOTE: Write 1 after Start Time was written to generate SYNC signal
</tr>
<tr>
    <td>
    <td>
    <td> 1
    <td> R/W
    <td> R/-
    <td> 0b
    <td> SYNC0 generation: <br/>
         0: Deactivated <br/>
         1: SYNC0 pulse is generated
</tr>
<tr>
    <td>
    <td>
    <td> 2
    <td> R/W
    <td> R/-
    <td> 0b
    <td> SYNC1 generation: <br/>
         0: Deactivated <br/>
         1: SYNC1 pulse is generated
</tr>
<tr>
    <td>
    <td>
    <td> 3-7
    <td> R/W
    <td> R/-
    <td> 00000b
    <td>
</tr>
<tr>
    <td> Pulse Length of SyncSignals
    <td> 0x0982-0x0983
    <td>
    <td>
    <td>
    <td>
    <td> **EEPROM reload operation updates this field (like ET1100,unlike IP Core)**
</tr>
<tr>
    <td>
    <td>
    <td> 0-15
    <td> R/-
    <td> **RW**-
    <td> 0
    <td> Pulse length of SyncSignals (in Units of 10ns) <br/>
         0: Acknowledge mode: SyncSignal will be cleared by reading SYNC0/SYNC1 Status register Loaded from EEPROM ADR: 0x2 (During reset/ reload command)
</tr>
<tr>
    <td> Activation Status
    <td> 0x0984
    <td>
    <td>
    <td>
    <td> 0
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0
    <td> R/-
    <td> R/-
    <td> 0b
    <td> SYNC0 activation state: <br/>
         0: First SYNC0 pulse is not pending <br/>
         1: First SYNC0 pulse is pending
</tr>
<tr>
    <td>
    <td>
    <td> 1
    <td> R/-
    <td> R/-
    <td> 0b
    <td> SYNC1 activation state: <br/>
         0: First SYNC1 pulse is not pending <br/>
         1: First SYNC1 pulse is pending
</tr>
<tr>
    <td>
    <td>
    <td> 2-7
    <td> R/-
    <td> R/-
    <td>
    <td> Reserved
</tr>
<tr>
    <td> SYNC0 Status
    <td> 0x098E
    <td>
    <td>
    <td>
    <td> 0
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0
    <td> R/-
    <td> R/-
    <td> 0b
    <td> SYNC0 state for Acknowledge mode.<br/>
         SYNC0 in Acknowledge mode is cleared by reading this register from PDI, use only in Acknowledge mode
</tr>
<tr>
    <td>
    <td>
    <td> 1-7
    <td> R/-
    <td> R/-
    <td>
    <td> Reserved
</tr>
<tr>
    <td> SYNC1 Status
    <td> 0x098F
    <td>
    <td>
    <td>
    <td> 0
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0
    <td> R/-
    <td> R/-
    <td> 0b
    <td> SYNC1 state for Acknowledge mode.<br/>
         SYNC1 in Acknowledge mode is cleared by reading this register from PDI, use only in Acknowledge mode
</tr>
<tr>
    <td>
    <td>
    <td> 1-7
    <td> R/-
    <td> R/-
    <td>
    <td> Reserved
</tr>
<tr>
    <td> Start Time Cyclic Operation
    <td> 0x0990-0x0997
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-63
    <td> R/W
    <td> R/-
    <td> 0
    <td> Write: Start time (System time) of cyclic operation <br/>
         Read: System time of next SYNC0 pulse
</tr>
<tr>
    <td> Next SYNC1 Pulse
    <td> 0x0998-0x099F
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-63
    <td> R/-
    <td> R/-
    <td> 0
    <td> System time of next SYNC1 pulse
</tr>
<tr>
    <td> SYNC0 Cycle Time
    <td> 0x09A0-0x09A3
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-31
    <td> R/W
    <td> R/-
    <td> 0
    <td> Time between two consecutive SYNC0 pulses in ns
         0: Single shot mode, generate only one SYNC0 pulse.
</tr>
<tr>
    <td> SYNC1 Cycle Time
    <td> 0x09A4-0x09A7
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-31
    <td> R/W
    <td> R/-
    <td> 0
    <td> Time between SYNC1 pulses and SYNC0 pulse in ns
</tr>
<tr>
    <td> Latch0 Control
    <td> 0x09A8
    <td>
    <td>
    <td>
    <td> 0
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0
    <td> R/W
    <td> R/-
    <td> 0b
    <td> Latch0 positive edge: <br/>
         0: Continuous Latch active <br/>
         1: Single event (only first event active)
</tr>
<tr>
    <td>
    <td>
    <td> 1
    <td> R/W
    <td> R/-
    <td> 0b
    <td> Latch0 negative edge
</tr>
<tr>
    <td>
    <td>
    <td> 2-7
    <td> R/W
    <td> R/-
    <td>
    <td> Reserved
</tr>
<tr>
    <td> Latch1 Control
    <td> 0x09A9
    <td>
    <td>
    <td>
    <td> 0
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0
    <td> R/W
    <td> R/-
    <td> 0b
    <td> Latch1 positive edge
</tr>
<tr>
    <td>
    <td>
    <td> 1
    <td> R/W
    <td> R/-
    <td> 0b
    <td> Latch1 negative edge
</tr>
<tr>
    <td>
    <td>
    <td> 2-7
    <td> R/W
    <td> R/-
    <td>
    <td> Reserved
</tr>
<tr>
    <td> Latch0 Status
    <td> 0x09AE
    <td>
    <td>
    <td>
    <td> 0
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0
    <td> R/-
    <td> R/-
    <td> 0b
    <td> Event Latch0 positive edge. <br/>
         0: Positive edge not detected or continuous mode <br/>
         1: Positive edge detected in single event mode only. <br/>
         Flag cleared by reading out Latch0 Time Positive Edge.
</tr>
<tr>
    <td>
    <td>
    <td> 1
    <td> R/-
    <td> R/-
    <td> 0b
    <td> Event Latch0 negative edge.
</tr>
<tr>
    <td>
    <td>
    <td> 2
    <td> R/-
    <td> R/-
    <td> 0b
    <td> Latch0 pin state **Not available in TI ESC**
</tr>
<tr>
    <td>
    <td>
    <td> 3-7
    <td> R/-
    <td> R/-
    <td>
    <td> Reserved
</tr>
<tr>
    <td> Latch1 Status
    <td> 0x09AF
    <td>
    <td>
    <td>
    <td> 0
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0
    <td> R/-
    <td> R/-
    <td> 0b
    <td> Event Latch1 positive edge.
</tr>
<tr>
    <td>
    <td>
    <td> 1
    <td> R/-
    <td> R/-
    <td> 0b
    <td>  Event Latch1 negative edge
</tr>
<tr>
    <td>
    <td>
    <td> 2
    <td> R/-
    <td> R/-
    <td> 0b
    <td> Latch1 pin state **Not available in TI ESC**
</tr>
<tr>
    <td>
    <td>
    <td> 3
    <td> R/-
    <td> R/-
    <td>
    <td> Reserved
</tr>
<tr>
    <td> Latch0 Time Positive Edge
    <td> 0x09B0-0x09B7
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-63
    <td> R/-
    <td> R/-
    <td> 0
    <td> Register captures System time at the positive edge of the Latch0 signal. Reading clears Latch0 Status 0x09AE[0]
</tr>
<tr>
    <td> Latch0 Time Negative Edge
    <td> 0x09B8-0x09BF
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-63
    <td> R/-
    <td> R/-
    <td> 0
    <td> System time at the negative edge of the Latch0 signal.
</tr>
<tr>
    <td> Latch1 Time Positive Edge
    <td> 0x09C0-0x09C7
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-63
    <td> R/-
    <td> R/-
    <td> 0
    <td> System time at the positive edge of the Latch1 signal.
</tr>
<tr>
    <td> Latch1 Time Negative Edge
    <td> 0x9C8-0x9CF
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-63
    <td> R/-
    <td> R/-
    <td> 0
    <td> System time at the negative edge of the Latch1 signal.
</tr>
<tr>
    <td> EtherCAT Buffer Change Event Time
    <td> 0x09F0-0x09F3
    <td>
    <td>
    <td>
    <td>
    <td> **Not available in TI ESC**
</tr>
<tr>
    <td> PDI Buffer Start Event Time
    <td> 0x09F8-0x9FB
    <td>
    <td>
    <td>
    <td>
    <td> **Not available in TI ESC**
</tr>
<tr>
    <td> PDI Buffer Change Event Time
    <td> 0x09FC-0x09FF
    <td>
    <td>
    <td>
    <td>
    <td> **Not available in TI ESC**
</tr>
<tr>
    <td colspan="7"> **TI ESC Vendor Specific Registers**
</tr>
<tr>
    <td> RX Port0 frame counter
    <td> 0x0E00-0x0E03
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-31
    <td> R/-
    <td> R/-
    <td> 0
    <td> Number of valid frames seen at port 0. <br/>
         Change in this register can be used to control the ACT LED.
</tr>
<tr>
    <td> RX Port1 frame counter
    <td> 0x0E04-0x0E07
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-31
    <td> R/-
    <td> R/-
    <td> 0
    <td> Number of valid frames seen at port 1. <br/>
         Change in this register can be used to control the ACT LED.
</tr>
<tr>
    <td> Port0 PHY address
    <td> 0x0E08
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-7
    <td> R/-
    <td> R/W
    <td> 0
    <td> To specify PHY address of PHY connected to physical port0 to PRU firmware
</tr>
<tr>
    <td> Port1 PHY address
    <td> 0x0E09
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-7
    <td> R/-
    <td> R/W
    <td> 0
    <td> To specify PHY address of PHY connected to physical port1 to PRU firmware
</tr>
<tr>
    <td> PDI ISR DIGIO pin selection register
    <td> 0x0E0A
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-7
    <td> R/W
    <td> R/W
    <td> 0
    <td> PDI ISR DIGIO pin selection register, selects one of pr1_edio_data_out(24+N), N=0-7 pins as PDI ISR hw pin, configure 255 to disable. Set corresponding bitmask to enable. Application needs to configure pinmux correctly for this to work. <br/>
</tr>
<tr>
    <td> PRU MII RX LINK Polarity
    <td> 0x0E0C
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-31
    <td> R/-
    <td> R/W
    <td> 0b
    <td> Link LED signal polarity PHY address N (Bit N): <br/>
         0 - Active high <br/>
         1 - Active low
</tr>
<tr>
    <td> Port0 TX Start Delay
    <td> 0x0E10
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-15
    <td> R/W
    <td> R/W
    <td> 0x50
    <td> TX_START_DELAY for port 0. Change from default values is not recommended.
</tr>
<tr>
    <td> Port1 TX Start Delay
    <td> 0x0E12
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-15
    <td> R/W
    <td> R/W
    <td> 0x50
    <td> TX_START_DELAY for port 1. Change from default values is not recommended.
</tr>
<tr>
    <td> ESC Reset
    <td> 0x0E14-0x0E17
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-31
    <td> R/W
    <td> R/W
    <td> 0
    <td> Vendor specific ESC WARM RESET register. Write "RST"(0x545352) or "rst"(0x747372) to force WARM RESET
</tr>
<tr>
    <td colspan="7"> NOTE: Normally one need not program any of these following registers(0x0E18-0x0E1F) and even access can be disabled via ECAT if not desired. This may be used to tweak DC drift compensation algorithm for PC based MainDevices (using higher ppm timer) as network clock reference instead of first DC enabled SubDevice in the network.
</tr>
<tr>
    <td> Aggressive DC drift compensation step register
    <td> 0x0E18
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-7
    <td> R/W
    <td> R/W
    <td> 1
    <td> Vendor specific aggressive DC drift compensation step register  1 by default, if System Difference exceeds 0xE1D:0xE1C, compensate by additional steps specified here.
</tr>
<tr>
    <td> Speed counter scale factor register
    <td> 0x0E19
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-7
    <td> R/W
    <td> R/W
    <td> 2
    <td> Speed counter is computed by default in 5ns units (this register is to scale this)
</tr>
<tr>
    <td> Drift compensation fast convergence
    <td> 0x0E1A
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-7
    <td> R/W
    <td> R/W
    <td> 0
    <td> Vendor specific DC drift compensation fast convergence register takes into account System time diff sign if enabled.<br/>
         0 : Enabled (default)<br/>
        Otherwise : Disabled
</tr>
<tr>
    <td> Vendor specific DC drift compensation adjust max limit. 32 (default)
    <td> 0x0E1B
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-7
    <td> R/W
    <td> R/W
    <td> 32
    <td> Vendor specific DC drift compensation adjust max limit. 32 (default)
</tr>
<tr>
    <td> Aggressive Drift compensation Trigger
    <td> 0x0E1C-0x0E1D
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-15
    <td> R/W
    <td> R/W
    <td> 255
    <td> Vendor specific aggressive DC drift compensation trigger register. 255 ns by default
</tr>
<tr>
    <td colspan="7"> NOTE: Normally one need not program the following register and even access can be disabled via ECAT if not desired. This register is used to enable/disable Enhanced Process Data access latency feature (must be enabled in the application as well).
</tr>
<tr>
    <td> Enhanced Process Data buffer access latency mode control Register
    <td> 0x0E24-0x0E25
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-15
    <td> R/W
    <td> R/W
    <td> 0
    <td> Vendor specific Enhanced Process Data buffer access latency mode control Register. <br/>
         0 by default.<br/>
         If set to 1 then firmware assumes that application has the Enhanced mode enabled and works accordingly. (Will not work correctly if not enabled from application side).
</tr>
<tr>
    <td> PHY RX Error Counter Register
    <td> 0x0E28-0x0E29
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-15
    <td> -
    <td> R/W
    <td> 0
    <td> Address of PHY register maintaining RX Error (RX_ERR) count during frame (when RX_DV is asserted). <br/>
         0 by default. If set to 0, no PHY register is read for updating 0x0301/0x0303 registers in TI ESC.<br/>
         For TI DP83869/DP83867/DP82826E/DP83822 PHYs, 0x15 should be configured.<br/>
        **NOTE: This register should be programmed before PRU cores start running** <br/>
</tr>
<tr>
    <td> PDI ISR Time Register
    <td> 0x0E2C-0x0E33
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-63
    <td> -
    <td> R/W
    <td> 0
    <td> Register captures the System time when PDI ISR is triggered. <br/>
</tr>
<tr>
    <td> PRU Clock Frequency Selection Register
    <td> 0x0E34-0x0E35
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-7
    <td> -
    <td> R/W
    <td> 0
    <td> Selection of 200 MHz or 333 MHz clock frequency for PRU-ICSS Core Clock and IEP Clock used by EtherCAT SubDevice Firmware. <br/>
         0 : 200 MHz (default)<br/>
         1 : 333 MHz <br/>
         **NOTE: This register should be programmed before PRU cores start running** <br/>
         **This feature is not available in SoCs with PRU-ICSSM (AM263x)**<br/>

</tr>
<!-- <tr>
    <td> MDIO Alive Register for MDIO Manual Mode
    <td> 0x0E48-0x0E4B
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-31
    <td> R/-
    <td> R/-
    <td> 0
    <td> MDIO Alive register when the MDIO Manual Mode is enabled (Enabled by default in SoCs with PRU-ICSSG version 1.3 (AM243x, AM64x)) <br/>
         Each of the 32 bits of this register is set if the most recent access to the PHY with address corresponding to the register bit number was acknowledged by the PHY, the bit is reset if the PHY fails to acknowledge the access. <br/>
         **This feature is not available in SoCs with PRU-ICSSM (AM263x)**
</tr> -->
<tr>
    <td> MDIO Link Register for MDIO Manual Mode
    <td> 0x0E4C-0x0E4F
    <td>
    <td>
    <td>
    <td>
    <td>
</tr>
<tr>
    <td>
    <td>
    <td> 0-31
    <td> R/-
    <td> R/-
    <td> 0
    <td> MDIO Link State register when the MDIO Manual Mode is enabled (Enabled by default in SoCs with PRU-ICSSG version 1.3 (AM243x, AM64x)) <br/>
         This register is updated after a read of the Generic Status Register of a PHY. The corresponding bit is set if the PHY with the corresponding address has link and the PHY acknowledges the read transaction. The bit is cleared to zero if the PHY indicates it does not have link or fails to acknowledge the read transaction. <br/>
         **This feature is not available in SoCs with PRU-ICSSM (AM263x)**<br/>
</tr>
<tr>
    <td> MDIO User Access 0 Register for MDIO Manual Mode
    <td> 0x0EC0-0x0EC3
    <td>
    <td>
    <td>
    <td>
    <td> MDIO User Access 0 Register when the MDIO Manual Mode is enabled (Enabled by default in SoCs with PRU-ICSSG version 1.3 (AM243x, AM64x)) <br/>
         **This feature is not available in SoCs with PRU-ICSSM (AM263x)**<br/>
</tr>
<tr>
    <td>
    <td>
    <td> 0-15
    <td> R/W
    <td> R/W
    <td> 0
    <td> The data value read from or to be written to the specified PHY register. <br/>
         **NOTE: This register should be programmed with PHY address for Port 0, if MDIO Manual Mode is being used (Mandatory for AM243x and AM64x)**<br/>
</tr>
<tr>
    <td>
    <td>
    <td> 16-20
    <td> R/W
    <td> R/W
    <td> 0
    <td> This field specifies the PHY to be accessed for this transaction.
</tr>
<tr>
    <td>
    <td>
    <td> 21-25
    <td> R/W
    <td> R/W
    <td> 0
    <td> This field specifies the PHY register to be accessed for this transaction.
</tr>
<tr>
    <td>
    <td>
    <td> 26-28
    <td> R/W
    <td> R/W
    <td> 0
    <td> Reserved
</tr>
<tr>
    <td>
    <td>
    <td> 29
    <td> R/W
    <td> R/W
    <td> 0
    <td> This bit is set if the PHY acknowledged the read transaction.
</tr>
<tr>
    <td>
    <td>
    <td> 30
    <td> R/W
    <td> R/W
    <td> 0
    <td> Setting this bit to a 1 causes the MDIO transaction to be a register write, otherwise it is a register read.
</tr>
<tr>
    <td>
    <td>
    <td> 31
    <td> R/W
    <td> R/W
    <td> 0
    <td> Go bit <br/>
         Writing a 1h to this bit triggers an MDIO access when it is convenient for it to do so, this is not an instantaneous process.<br/>
         Writing a 0h to this bit has no effect. <br/>
         This bit will self clear when the requested access has been completed. <br/>
         **NOTE: Any writes to this register should be avoided when the go bit is 1h. If byte access is being used, the go bit should be written last.**<br/>
</tr>
<tr>
    <td> MDIO User PHY Select 0 Register for MDIO Manual Mode
    <td> 0x0EC4-0x0EC7
    <td>
    <td>
    <td>
    <td>
    <td> MDIO User PHY Select 0 Register when the MDIO Manual Mode is enabled (Enabled by default in SoCs with PRU-ICSSG version 1.3 (AM243x, AM64x)) <br/>
         **This feature is not available in SoCs with PRU-ICSSM (AM263x)**<br/>
</tr>
<tr>
    <td>
    <td>
    <td> 0-4
    <td> R/W
    <td> R/W
    <td> 0
    <td> PHY address whose link status is to be monitored <br/>
         **NOTE: This register should be programmed with PHY address for Port 0, if MDIO Manual Mode is being used (Mandatory for AM243x and AM64x)**<br/>
</tr>
<tr>
    <td>
    <td>
    <td> 5
    <td> R/W
    <td> R/W
    <td> 0
    <td> Reserved
</tr>
<tr>
    <td>
    <td>
    <td> 6
    <td> R/W
    <td> R/W
    <td> 0
    <td> Link change interrupt enable <br/>
         Set to 1h to determine link status using the MLINK pin <br/>
        **NOTE: This field should be set to 1, if MDIO Manual Mode is being used (Mandatory for AM243x and AM64x)**<br/>

</tr>
<tr>
    <td>
    <td>
    <td> 7
    <td> R/W
    <td> R/W
    <td> 0
    <td> Link status determination select <br/>
         Set to 1h to enable link change status interrupts for PHY address specified in bits 0-4. Link change interrupts are disabled if this bit is set to 0h. <br/>
         **NOTE: This field should be set to 1, if MDIO Manual Mode is being used (Mandatory for AM243x and AM64x)**<br/>
</tr>
<tr>
    <td>
    <td>
    <td> 8-31
    <td> R/W
    <td> R/W
    <td> 0
    <td> Reserved
</tr>
<tr>
    <td> MDIO User Access 1 Register for MDIO Manual Mode
    <td> 0x0EC8-0x0ECB
    <td>
    <td>
    <td>
    <td>
    <td> MDIO User Access 1 Register when the MDIO Manual Mode is enabled (Enabled by default in SoCs with PRU-ICSSG version 1.3 (AM243x, AM64x)) <br/>
         **This feature is not available in SoCs with PRU-ICSSM (AM263x)**<br/>
</tr>
<tr>
    <td>
    <td>
    <td> 0-15
    <td> R/W
    <td> R/W
    <td> 0
    <td> The data value read from or to be written to the specified PHY register. <br/>
         **NOTE: This register should be programmed with PHY address for Port 0, if MDIO Manual Mode is being used (Mandatory for AM243x and AM64x)**<br/>
</tr>
<tr>
    <td>
    <td>
    <td> 16-20
    <td> R/W
    <td> R/W
    <td> 0
    <td> This field specifies the PHY to be accessed for this transaction.
</tr>
<tr>
    <td>
    <td>
    <td> 21-25
    <td> R/W
    <td> R/W
    <td> 0
    <td> This field specifies the PHY register to be accessed for this transaction.
</tr>
<tr>
    <td>
    <td>
    <td> 26-28
    <td> R/W
    <td> R/W
    <td> 0
    <td> Reserved
</tr>
<tr>
    <td>
    <td>
    <td> 29
    <td> R/W
    <td> R/W
    <td> 0
    <td> This bit is set if the PHY acknowledged the read transaction.
</tr>
<tr>
    <td>
    <td>
    <td> 30
    <td> R/W
    <td> R/W
    <td> 0
    <td> Setting this bit to a 1 causes the MDIO transaction to be a register write, otherwise it is a register read.
</tr>
<tr>
    <td>
    <td>
    <td> 31
    <td> R/W
    <td> R/W
    <td> 0
    <td> Go bit <br/>
         Writing a 1h to this bit triggers an MDIO access when it is convenient for it to do so, this is not an instantaneous process.<br/>
         Writing a 0h to this bit has no effect. <br/>
         This bit will self clear when the requested access has been completed. <br/>
         **NOTE: Any writes to this register should be avoided when the go bit is 1h. If byte access is being used, the go bit should be written last.**<br/>
</tr>
<tr>
    <td> MDIO User PHY Select 1 Register for MDIO Manual Mode
    <td> 0x0ECC-0x0ECF
    <td>
    <td>
    <td>
    <td>
    <td> MDIO User PHY Select 1 Register when the MDIO Manual Mode is enabled (Enabled by default in SoCs with PRU-ICSSG version 1.3 (AM243x, AM64x)) <br/>
         **This feature is not available in SoCs with PRU-ICSSM (AM263x)**
</tr>
<tr>
    <td>
    <td>
    <td> 0-4
    <td> R/W
    <td> R/W
    <td> 0
    <td> PHY address whose link status is to be monitored <br/>
         **NOTE: This register should be programmed with PHY address for Port 1, if MDIO Manual Mode is being used (Mandatory for AM243x and AM64x)**<br/>
</tr>
<tr>
    <td>
    <td>
    <td> 5
    <td> R/W
    <td> R/W
    <td> 0
    <td> Reserved
</tr>
<tr>
    <td>
    <td>
    <td> 6
    <td> R/W
    <td> R/W
    <td> 0
    <td> Link change interrupt enable <br/>
         Set to 1h to determine link status using the MLINK pin <br/>
        **NOTE: This field should be set to 1, if MDIO Manual Mode is being used (Mandatory for AM243x and AM64x)**<br/>

</tr>
<tr>
    <td>
    <td>
    <td> 7
    <td> R/W
    <td> R/W
    <td> 0
    <td> Link status determination select <br/>
         Set to 1h to enable link change status interrupts for PHY address specified in bits 0-4. Link change interrupts are disabled if this bit is set to 0h.<br/>
         **NOTE: This field should be set to 1, if MDIO Manual Mode is being used (Mandatory for AM243x and AM64x)**<br/>
</tr>
<tr>
    <td>
    <td>
    <td> 8-31
    <td> R/W
    <td> R/W
    <td> 0
    <td> Reserved
</tr>
</table>
