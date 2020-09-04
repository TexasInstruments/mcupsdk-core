# Benchmark Demo with GUI Composer {#EXAMPLE_MOTORCONTROL_BENCHMARKDEMO}

[TOC]

## Introduction

The Benchmark Demo showcases the capabilities of the 4 R5F cores on AM243x. Each R5F core runs a particular benchmark test and displays the results on a Cloud-based GUI.

The @VAR_BOARD_NAME version of the demo also has power monitoring functionality and displays voltage, current, and total power consumption.

The mapping of R5F core to the benchmark that is run is shown below
<table>
<tr>
    <th>R5 Core
    <th>Benchmark
	<th>Default Option
	<th>Description
</tr>
<tr>
    <td>R5F Core 0 (rf5ss0-0)</td>
    <td>Stream memory benchmark</td>
	<td>1 hz execution loop</td>
	<td>Memory bandwidth benchmark for R5. For more details on the STREAM benchmark, see http://www.cs.virginia.edu/stream/ref.html
    </td>
</tr>
<tr>
    <td>R5F Core 1 (rf5ss0-1)</td>
    <td>CFFT benchmark</td>
	<td>1Khz execution loop, 128 point CFFT</td>
	<td>CMSIS CFFT benchmark for R5F</td>
</tr>
<tr>
    <td>R5F Core 2 (rf5ss1-0)</td>
    <td>FIR filtering benchmark</td>
	<td>1 Khz execution loop, 320 point, 29-tap</td>
	<td>CMSIS FIR filtering benchmark for R5F </td>
</tr>
<tr>
    <td>R5F Core 3 (rf5ss1-1)</td>
    <td>Field Oriented Control (FOC) benchmark </td>
	<td>32 Khz execution loop</td>
	<td>CMSIS FOC benchmark for R5F</td>
</tr>
</table>

# Steps to Run the Example {#STEPS}

## EVM Setup

- To run the demo from GUI composer, firstly you need to make sure the EVM is setup as mentioned here, \ref EVM_SETUP_PAGE
- Specifically,
  - Make sure you have connected the power, USB UART and USB JTAG cables as mentioned here, \ref EVM_CABLES
  - Make sure you have identified the UART port that is used for the print logs as mentioned here, \ref CCS_UART_TERMINAL
  - Check if the SOC initialization binary is already flashed. If flashed, you will see something like below in the UART terminal,
    \code
    Starting NULL Bootloader ...

    DMSC Firmware Version 21.1.1--v2021.01a (Terrific Lla
    DMSC Firmware revision 0x15
    DMSC ABI revision 3.1

    INFO: Bootloader_runCpu:147: CPU r5f1-0  is initialized to 800000000 Hz !!!
    INFO: Bootloader_runCpu:147: CPU r5f1-1 is initialized to 800000000 Hz !!!
    INFO: Bootloader_runCpu:147: CPU m4f0-0 is initialized to 400000000 Hz !!!
    INFO: Bootloader_loadSelfCpu:199: CPU r5f0-0 is initialized to 800000000 Hz !!!
    INFO: Bootloader_loadSelfCpu:199: CPU r5f0-1 is initialized to 800000000 Hz !!!
    INFO: Bootloader_runSelfCpu:209: All done, reseting self ...
    \endcode
  - If you see this in the UART terminal, you are all set to launch the demo, else you will
    need to flash the SOC initialization binary as mentioned here, \ref EVM_FLASH_SOC_INIT
  - Now you can launch the demo via GUI composer as mentioned next

## Launch the demo

- Close all open UART terminals and JTAG connections. GUI composer via the browser will need access
  to the UART port and JTAG port.

- Power on the EVM with the USB UART and USB JTAG cables connected.

- In a web browser, Firefox, Chrome or Edge, click on the below link
  - For @VAR_LP_BOARD_NAME, https://dev.ti.com/gallery/view/SitaraMCU/AM243x_LaunchPad_OOB_Experience
  - For @VAR_BOARD_NAME, https://dev.ti.com/gallery/view/SitaraMCU/AM243x_EVM_OOB_Experience

- Click on benchmark demo

    \imageStyle{am243x_benchmark_demo_00.png,width:20%}
    \image html am243x_benchmark_demo_00.png

- Select the UART port noted during EVM setup and set the UART baud rate as shown below,

    \imageStyle{am243x_benchmark_demo_01.png,width:20%}
    \image html am243x_benchmark_demo_01.png

    \imageStyle{am243x_benchmark_demo_02.png,width:30%}
    \image html am243x_benchmark_demo_02.png

- Click on `LAUNCH` to launch the demo

    \imageStyle{am243x_benchmark_demo_03.png,width:50%}
    \image html am243x_benchmark_demo_03.png

- You will see some setup happening in the bottom of the browser window, let this continue. First time this make take a minute or so to complete.

    \imageStyle{am243x_benchmark_demo_04.png,width:90%}
    \image html am243x_benchmark_demo_04.png

- After all the setup and download is done, you will see the benchmark demo results updating in the GUI as shown below

    \imageStyle{am243x_benchmark_demo_05.png,width:90%}
    \image html am243x_benchmark_demo_05.png

- You can interact with the GUI options to change the settings and see the updated results.

# Supported Combinations

\cond SOC_AM64X || SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss0-1 nortos
 ^              | r5fss1-0 nortos
 ^              | r5fss1-1 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/motor_control/benchmark_demo/

\endcond

# Build and run the demo without GUI composer

\note The GUI composer loads a pre-built binary from a server to run the demo. This prebuilt binary cannot be changed by end users.

\note This section is a reference for users to understand and build the underlying source code of the demo. However
      they will not be able to see the results of the updated binary on the GUI composer.

\note This is a `system` or multi-core project, so refer to system project build instructions for CCS project or makefiles when building the example.

- **When using CCS projects to build**, import the system CCS project
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE). This will build all the dependant CPU projects as well
- **When using makefiles to build**, build the system makefile using
  make command (see \ref MAKEFILE_BUILD_PAGE). This will build all the dependant CPU makefiles as well.
- Launch a CCS debug session and run the executables, see \ref CCS_LAUNCH_PAGE
- This is a multi-core example. Hence the executables should be loaded and run for all the above mentioned cores
- When you run the demo it will output some text on UART console continuously. When running
  with GUI composer this text is used as input to render the contents of the GUI.


