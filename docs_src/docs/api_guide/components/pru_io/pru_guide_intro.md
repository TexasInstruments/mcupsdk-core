# Introduction {#COMPONENT_PRU_GUIDE_INTRO}

[TOC]

\cond SOC_AM64X || SOC_AM243X
## PRU_ICSSG Subsystem

The Programmable Real-Time Unit Subsystem and Industrial Communication Subsystem (PRU_ICSSG)
consists of:
* Two 32-bit load/store RISC CPU cores — Programmable Real-Time Units (PRU0 and PRU1)
* Two auxiliary 32-bit load/store RISC CPU cores — Auxiliary Programmable Real-Time Units (RTU_PRU0 and RTU_PRU1)
* Two 32-bit load/store RISC CPU cores — Transmit Programmable Real-Time Units (TX_PRU0 and TX_PRU1)
* Data RAMs per PRU core
* Instruction RAMs per PRU, per RTU_PRU and per TX_PRU cores
* Shared RAM
* Peripheral modules: UART0, ECAP0, PWM, MDIO, IEP0 and IEP1
* Interrupt controller (INTC) per ICSS subsystem and Task Manager per core

The programmable nature of the PRU cores, along with their access to pins, events and all device resources,
provides flexibility in implementing fast real-time responses, specialized data handling operations, custom
peripheral interfaces, and in offloading tasks from the other processor cores of the device.
The PRU cores are programmed with a small, deterministic instruction set. Each PRU can operate
independently or in coordination with each other and can also work in coordination with the device-level host
CPU. This interaction between processors is determined by the nature of the firmware loaded into the PRU’s
instruction memory.

  \imageStyle{ICSSG_block_diagram.png,width:85%}
  \image html ICSSG_block_diagram.png "PRU-ICSSG Sub-System Block Diagram"

Refer to latest AM64x TRM (Technical Reference Manual) for complete details. (Can be found here: <https://www.ti.com/product/AM6442>)

\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
## PRU_ICSSM Subsystem

The Programmable Real-Time Unit Subsystem and Industrial Communication Subsystem (PRU_ICSSM)
consists of:
* Two 32-bit load/store RISC CPU cores — Programmable Real-Time Units (PRU0 and PRU1)
* Data RAMs per PRU core
* Instruction RAMs per PRU
* Shared RAM
* Peripheral modules: UART0, ECAP0, MDIO, IEP0 and IEP1 
* Interrupt controller (INTC) per ICSS subsystem

The programmable nature of the PRU cores, along with their access to pins, events and all device resources,
provides flexibility in implementing fast real-time responses, specialized data handling operations, custom
peripheral interfaces, and in offloading tasks from the other processor cores of the device.
The PRU cores are programmed with a small, deterministic instruction set. Each PRU can operate
independently or in coordination with each other and can also work in coordination with the device-level host
CPU. This interaction between processors is determined by the nature of the firmware loaded into the PRU’s
instruction memory.

  \imageStyle{ICSSM_block_diagram.png,width:85%}
  \image html ICSSM_block_diagram.png "PRU-ICSSM Sub-System Block Diagram"

Refer to latest AM263x TRM (Technical Reference Manual) for complete details. (Can be found here: <https://www.ti.com/product/AM2634-Q1>)

\endcond

## PRU Core

PRU core is optimized for low latency and jitter. It contains:

* Non pipelined CPU which is 100% deterministic – no jitter in real-time execution
* Broadside interface with 1024 bit wide data bus supports lowest latency transfers
* Register mapped IOs and bit-wise addressing provide max interface flexibility
* Programmed in assembly or C language (or mixed C and inline assembly) with full support in CCS for source level debugging

  \imageStyle{PRU_core.png,width:50%}
  \image html PRU_core.png " "

## PRU Instruction Set

* <span style="color:green">Arithmetic Operations (green)</span>
* <span style="color:blue">Logic Operations (blue)</span>
* <span style="color:black">IO Operations (black)</span>
* <span style="color:red">Program Flow Control (red)</span>

<table>
  <tr>
    <td><span style="color:green">ADD</span> <td><span style="color:green">ADC</span> <td><span style="color:green">SUB</span>   <td><span style="color:green">SUC</span>  <td><span style="color:green">RSB</span>  <td><span style="color:green">RSC</span>
  <tr>
    <td><span style="color:blue">LSL</span> <td><span style="color:blue">LSR</span> <td><span style="color:blue">AND</span>   <td><span style="color:blue">OR</span>   <td><span style="color:blue">XOR</span>  <td><span style="color:blue">NOT</span>  <td><span style="color:blue">MIN</span>  <td><span style="color:blue">MAX</span>  <td><span style="color:blue">CLR</span>  <td><span style="color:blue">SET</span>  <td><span style="color:blue">LMBD</span>
  <tr>
    <td><span style="color:black">MOV</span> <td><span style="color:black">LDI</span> <td><span style="color:black">LDI32</span> <td><span style="color:black">LBBO</span> <td><span style="color:black">SBBO</span> <td><span style="color:black">LBCO</span> <td><span style="color:black">SBCO</span> <td><span style="color:black">MVIB</span> <td><span style="color:black">MVIW</span> <td><span style="color:black">MVID</span> <td><span style="color:black">ZERO</span> <td><span style="color:black">XIN</span>  <td><span style="color:black">XOUT</span> <td><span style="color:black">TSEN</span>
  <tr>
    <td><span style="color:red">JAL</span> <td><span style="color:red">JMP</span> <td><span style="color:red">QBGT</span> <td><span style="color:red">QBGE</span> <td><span style="color:red">QBLT</span> <td><span style="color:red">QBLE</span> <td><span style="color:red">QBEQ</span> <td><span style="color:red">QBNE</span> <td><span style="color:red">QBA</span>  <td><span style="color:red">QBBS</span> <td><span style="color:red">QBBC</span> <td><span style="color:red">WBS</span>  <td><span style="color:red">WBC</span> <td><span style="color:red">HALT</span> <td><span style="color:red">SLP</span>
</table>

Refer for usage details: [PRU Assembly Instruction User Guide](https://www.ti.com/lit/ug/spruij2/spruij2.pdf)
\cond SOC_AM64X || SOC_AM243X
* TSEN Instruction is used to enable task manager functionality.
Usage:

\code
    TSEN    1  ; Enable Task Manager
    TSEN    0  ; Disable Task Manager
\endcode
\endcond

## PRU Register Addressing

Possible Addressability in Instructions: Bit, Byte (8-bit), Half-word (16-bit), Word (32-bit), Pointer

  \imageStyle{Pru_register_addressing.png,width:55%}
  \image html Pru_register_addressing.png " "

Sample assembly code instructions:
\code
    ldi     r0.b0, 0x18         ; Load Immediate  - loads 0x18 value into r0.b0
    ldi     r0.b2, 0x24         ; Load Immediate  - loads 0x24 value into r0.b2
    mov     r0.w1, r1.w0        ; Copy Value      - copies r1.w0 into r0.w1
    set     r1.b1, r1.b0, 5     ; Set Bit         - copies value of r1.b0 into r1.b1 after setting it's 5th bit (0 indexing)
    lbbo    &r3, r1, r2.w0, b0  ; Load Byte Burst - Copy "r0.b0" bytes into r3 from the memory address r1+r2.w0
\endcode

## PRU Broadside Accelerators

\cond SOC_AM64X || SOC_AM243X
PRU_ICSSG supports a broadside interface, which uses the XFR (XIN, XOUT, or XCHG) instruction to transfer the contents of PRUn, RTU_PRUn or TX_PRUn (where n = 0 or 1) registers to or from accelerators. This interface enables up to 30 registers (R0-R29, or 120 bytes) to be transferred in a single instruction.
  \imageStyle{Pru_broadside.png,width:90%}
  \image html Pru_broadside.png " "
\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
PRU_ICSSM supports a broadside interface, which uses the XFR (XIN, XOUT, or XCHG) instruction to transfer the contents of PRUn(where n = 0 or 1) registers to or from accelerators. This interface enables up to 30 registers (R0-R29, or 120 bytes) to be transferred in a single instruction.

Data processing accelerators
* MAC
* CRC16/32

Data movement accelerators
* XFR2VBUS
\endcond

## PRU I/O Control Modes

Each slice is directly connected to 20 GPIO pins which can be controlled by the PRUs using **R30** register and pin values can be read using **R31** register. By default **PRU0** and **PRU1** cores are able to control the respective GPIO pins (after they have been correctly pinmuxed).
Overall:
* PRU has full input and output control on all interfaces
\cond SOC_AM64X || SOC_AM243X
* RTU_PRU and TX_PRU see R31 input and can process receive in parallel (If PRU_ICSSG slice is in GPIO mode, then RTU_PRU and TX_PRU both get the same r31_status (R31 register value) as PRU, but, only PRU core has control over pins, RTU_PRU/TX_PRU only have observation or input. RTU_PRU and TX_PRU has no support for GPO)
* PRU/RTU_PRU supports load sharing option for sigma-delta and encoder interface (Load sharing across 3 PRU cores per slice of ICSSG which enables RTU (SD0-SD2), PRU (SD3-SD5), TX_PRU (SD6-SD8) control 3 channels each).

  \imageStyle{Pru_gpio.png,width:30%}
  \image html Pru_gpio.png "GPIO Connection"
\endcond
  <table>
    <tr><th> Direct Output            <td> 20 GPOs R30[19-0]
    <tr><th> Direct Input             <td> 20 GPIs R31[19-0]
    <tr><th> 16 bit parallel capture  <td> 16 GPIs R31[15-0], CLK R31[16]
    <tr><th> Serial shift out mode    <td> DATAOUT GPO0, CLKOUT GPO1, two 16 bit shadow register, core_clock based
    <tr><th> Serial shift in mode     <td> DATAIN GPO0, 28 bit shift register, CNT_16 flag, core_clock with two fractional divider 1, 1.5, 2 …
    <tr><th> 9 x Sigma Delta          <td> sinc1/2/3 accumulators, OSR4..256, fast detect
    <tr><th> 3 x Peripheral Interface <td> encoder (RS-485) interface
  </table>

\cond SOC_AM64X || SOC_AM243X
## Real-Time Task Manager

The PRU_ICSSG system integrates 6 Task Managers used for efficient switching between tasks.
There is one dedicated task manager for each PRU, RTU and TX_PRU core (PRU0/ PRU1, RTU_PRU0/
RTU_PRU1 and TX_PRU0/ TX_PRU1 cores), which are described in the subsequent sections. Each task
manager works independently from the others.

* Allows change of Program Counter in 2 cycles
* Flexible context save/restore using SPAD register banks
* 2 levels of pre-emption, 5 sub-tasks per level
* 151 triggers from internal and external events
    * Ethernet receive/transmit/link states
    * IEP timer capture/compare/SYNC_OUT
    * INTC HOST_OUT[7:0], HOST_TASKMGR[7:0]
    * SPINLOCK_FEED_REQ[63:0]
    * PRU_INTR_IN[2:0]
    * xfr2vbus DMA data ready

  \imageStyle{Pru_task_manager.png,width:30%}
  \image html Pru_task_manager.png " Task Manager Block Diagram"
\endcond
