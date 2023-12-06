# PCIE buffer transfer RC {#EXAMPLES_DRIVERS_PCIE_BUF_TRANSFER_RC}

[TOC]

# Introduction

The PCIe buffer transfer (RC) example demonstrates a buffer transfer between RC and EP device.

The RC device sends a buffer to the EP device, and the EP device loops back the same buffer to RC. RC device validates the buffer transfer by comparing the buffer received from EP with the original source buffer.

# Supported Combinations

\cond SOC_AM64X || SOC_AM243X || SOC_AM65X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/pcie/pcie_buf_transfer/pcie_buf_transfer_rc

\endcond

# Steps to Run the Example

## Build the example

- When using CCS projects to build, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- When using makefiles to build, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)

## HW Setup

\if SOC_AM65X\note Make sure you have setup the IDK with cable connections as shown here, \ref IDK_SETUP_PAGE.
      In addition do below steps.
\else
\note Make sure you have setup the EVM with cable connections as shown here, \ref EVM_SETUP_PAGE.
      In addition do below steps.
\endif

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

\cond SOC_AM65X

\cond SOC_AM65X
### AM65X-IDK
\endcond

- For connecting two board in RC and EP mode a specialized cable as below is required
    \imageStyle{pcie_cable_am65x.png, width:50%}
    \image html pcie_cable_am65x.png
- This cable can be obtained from Adex Electronics (https://www.adexelec.com).
- Modify the cable to remove resistors in CK+ and CK- in order to avoid ground loops (power) and smoking clock drivers (clk+/-).
- The ends of the modified cable should look like below:
    - A side
        \imageStyle{pcie_cable_am65x_a1.png,width:90%}
        \imageStyle{pcie_cable_am65x_a2.png,width:90%}

        <table style="border: 0 px">
        <tr>
            <td> \image html pcie_cable_am65x_a1.png "PCIe cable A side end 1" </td>
            <td> \image html pcie_cable_am65x_a2.png "PCIe cable A side end 2" </td>
        </tr>
        </table>
    - B side
        \imageStyle{pcie_cable_am65x_b1.png,width:90%}
        \imageStyle{pcie_cable_am65x_b2.png,width:90%}

        <table style="border: 0 px">
        <tr>
            <td> \image html pcie_cable_am65x_b1.png "PCIe cable B side end 1" </td>
            <td> \image html pcie_cable_am65x_b2.png "PCIe cable B side end 2" </td>
        </tr>
        </table>
\endcond

## Run the example
- Launch a CCS debug session and run the example executable, see CCS Launch, Load and Run
- Run the \ref EXAMPLES_DRIVERS_PCIE_BUF_TRANSFER_EP example on the other connected board
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
