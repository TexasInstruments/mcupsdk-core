# PCIE MSI IRQ RC {#EXAMPLES_DRIVERS_PCIE_MSI_IRQ_RC}

[TOC]

# Introduction

The PCIe MSI interrupt (RC) example demonstrates an EP device interrupting RC device using MSI (Message Signalled Interrupt).

The RC device sends a buffer to the EP device, and the EP device loops back the same buffer to RC and sends an MSI to RC. RC device will wait for the interrupt from EP and validates the buffer transfer by comparing the buffer received from EP with the original source buffer.

\note The MSI makes use of Ring Accelerators in background to route interrupts to core. So an inbound region for the Ring Accelerator (tail) needs to be set for MSI in the RC device.

# Supported Combinations

\cond SOC_AM64X || SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/pcie/pcie_msi_irq/pcie_msi_irq_rc

\endcond

# Steps to Run the Example

## Build the example

- When using CCS projects to build, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- When using makefiles to build, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)

## HW Setup

\note Make sure you have setup the EVM with cable connections as shown here, \ref EVM_SETUP_PAGE.
      In addition do below steps.

\cond SOC_AM64X || SOC_AM243X

\cond SOC_AM243X
### AM243-EVM
\endcond
\cond SOC_AM64X
### AM64X-EVM
\endcond

- For connecting two board in RC and EP mode a specialized cable as below is required
    \imageStyle{pcie_cable.png, width:50%}
    \image html pcie_cable.png
- This cable can be obtained from Adex Electronics (https://www.adexelec.com).
- Modify the cable to remove resistors in CK+ and CK- in order to avoid ground loops (power) and smoking clock drivers (clk+/-).
- The ends of the modified cable should look like below:
    - A side
        \imageStyle{pcie_cable_a1.png,width:90%}
        \imageStyle{pcie_cable_a2.png,width:90%}

        <table style="border: 0 px">
        <tr>
            <td> \image html pcie_cable_a1.png "PCIe cable A side end 1" </td>
            <td> \image html pcie_cable_a2.png "PCIe cable A side end 2" </td>
        </tr>
        </table>
    - B side
        \imageStyle{pcie_cable_b1.png,width:90%}
        \imageStyle{pcie_cable_b2.png,width:90%}

        <table style="border: 0 px">
        <tr>
            <td> \image html pcie_cable_b1.png "PCIe cable B side end 1" </td>
            <td> \image html pcie_cable_b2.png "PCIe cable B side end 2" </td>
        </tr>
        </table>
\endcond

## Run the example
- Launch a CCS debug session and run the example executable, see CCS Launch, Load and Run
- Run the \ref EXAMPLES_DRIVERS_PCIE_MSI_IRQ_EP example on the other connected board
- You will see logs in the UART terminal as shown in the next section.

# See Also

\ref DRIVERS_PCIE_PAGE
## Sample output

\code

Device in RC mode
Endpoint Device ID: 100X
Endpoint Vendor ID: 17CDX
All tests have passed!!

\endcode
