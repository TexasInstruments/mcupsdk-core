# PCIE Benchmark RC {#EXAMPLES_DRIVERS_PCIE_BENCHMARK_RC}

[TOC]

# Introduction

The PCIe benchmark (RC) example benchmarks the buffer transfer speed between RC and EP device.

The RC device sends a 4MB buffer to the EP device. The EP device waits for the buffer to receive and acknowledges the reception of buffer by sending a legacy IRQ to the RC device. This is repeated multiple times using CPU transfer and UDMA and benchmarking is done from RC device.

# Supported Combinations

\cond SOC_AM64X || SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/pcie/pcie_benchmark/pcie_benchmark_rc

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
- Run the \ref EXAMPLES_DRIVERS_PCIE_BENCHMARK_EP example on the other connected board
- You will see logs in the UART terminal as shown in the next section.

# See Also

\ref DRIVERS_PCIE_PAGE
## Sample output

\code

Device in RC mode

Starting PCIe Buffer transfer using CPU
Time taken for transfer of 640MB buffer using CPU copy --> 237337409us
Speed of PCIe transfer in MB/s using CPU copy --> 2.697 MB/s

Starting PCIe Buffer transfer using UDMA
Time taken for transfer of 640MB buffer using UDMA --> 4842780us
Speed of PCIe transfer in MB/s using UDMA --> 132.155 MB/s

PCIE benchmark example completed!!
All tests have passed!!

\endcode
