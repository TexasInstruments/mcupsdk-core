# Benchmark Demo with Linux on A53 {#EXAMPLE_MOTORCONTROL_BENCHMARKDEMO}

[TOC]

## Introduction

The benchmark demo showcases the capabilities of the four R5F cores as well as
the A53 core. This demo is run by default when **Processor SDK Linux** is flashed to SD card.
Different R5F benchmarks will run on each of the four R5F cores. There are four options available for each
benchmark. Users can switch the options using the drop-down menu in the GUI to view
performance of R5F with each option.

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
    <td>ADC and PWM benchmark</td>
	<td>8 Khz execution loop</td>
	<td>ADC/PWM benchmark for R5F, here we read 5 ADC samples and do 1 PWM write. **NOTE: This is not enabled in current release**.
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
	<td>16 Khz execution loop</td>
	<td>CMSIS FOC benchmark for R5F</td>
</tr>
</table>

# Supported Combinations

\cond SOC_AM64X || SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss0-1 nortos
 ^              | r5fss1-0 nortos
 ^              | r5fss1-1 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/motor_control/benchmark_demo/

\endcond

# Steps to Run the Example {#STEPS}

## Hardware Prerequisites

Other than the basic EVM setup mentioned in \ref EVM_SETUP_PAGE, below additional HW is required to run this demo
-  PC with a web browser and wired ethernet connectivity (Windows or Linux)
-  Ethernet cables
-  Ethernet switch or ethernet router with DHCP service
-  SD card (minimum 2GB)

## Build the R5F side projects

\note The default **Processor SDK Linux** has prebuilt applications already, so this step can be skipped unless you have modified the
      demo application on R5F.

\note This is a `system` or multi-core project, so refer to system project build instructions for CCS project or makefiles when building the example.

- **When using CCS projects to build**, import the system CCS project
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE). This will build all the dependant CPU projects as well
- **When using makefiles to build**, build the system makefile using
  make command (see \ref MAKEFILE_BUILD_PAGE). This will build all the dependant CPU makefiles as well.
- After the build is complete the copy the below files to the SD card containing Linux filesystem in folder `/lib/firmware`.
  \code
  {system project output folder}/am64-main-r5f0_0-fw
  {system project output folder}/am64-main-r5f0_1-fw
  {system project output folder}/am64-main-r5f1_0-fw
  {system project output folder}/am64-main-r5f1_1-fw

  When using makefiles to build, "system project output folder", is
  ${SDK_INSTALL_PATH}/examples/motor_control/benchmark_demo/{board}/system_nortos

  When using CCS projects to build, "system project output folder", is
  ${CCS_WORKSPACE_PATH}/benchmark_demo_{board}_system_nortos/{Debug or Release}

  \endcode

- **NOTE**, the copy step can only be done using a Linux machine since the Linux filesystem on the SD card cannot be seen on a Windows machine.

## Run the demo

- To run this demo, Linux needs to run on the Cortex A-core.
  - This typically involves flashing a prebuilt Linux filesystem (`.wic` file) to the SD card
  - Refer to **Processor SDK Linux** user guide to setup Linux on Cortex-A core.

- Keep the EVM in **POWER-OFF** state.

- Insert the SD card with Linux filesystem onto the EVM

- Set the EVM boot mode to "SD boot mode"

  \imageStyle{boot_pins_sd_mode_oob.png,width:50%}
  \image html boot_pins_sd_mode_oob.png "SD BOOT MODE"

- Connect an ethernet cable from your ethernet switch or router to the EVM

- Connect your PC to the same ethernet switch or router

- Identify and setup a UART terminal console as mentioned in \ref CCS_UART_TERMINAL

- **POWER-ON** the EVM.

- After the Linux boot completes, login as "root", then type in "ifconfig" in Linux prompt to find out the IP address for the EVM

    \imageStyle{OOB_Linux_ifconfig.png,width:40%}
    \image html OOB_Linux_ifconfig.png

- On the PC, open a Internet Browser and enter in below in the address bar,
     \code
     http://{IP address of the EVM}:8081

     Example,
     http://192.168.86.198:8081
     \endcode


- The following web page will show in the browser,

    \imageStyle{OOB_GUI_root.png,width:50%}
    \image html OOB_GUI_root.png

- Click on the "Benchmark Demo",

    \imageStyle{OOB_GUI_stats.png,width:70%}
    \image html OOB_GUI_stats.png

- The R5F application for each core is fixed. User can change the `Options` for each core, then click `LOAD` to refresh the statistics


# Sample Output

Shown below is a sample output when the application is run.

\imageStyle{OOB_GUI_stats.png,width:70%}
\image html OOB_GUI_stats.png

# Benchmark Results

\cond SOC_AM64X

 CORE           | CMSIS Application | CPU Usage(Avg) | Interrupt Latency (ns)(Avg) | Cycle Counter per loop(Avg)
 ---------------|-------------------|----------------|-----------------------------|----------------------------
 R5FSS0_1       | CFFT(128pt)       |     0          |          358                |         7878
 ^              | CFFT(256pt)       |     2          |          393                |         18374
 ^				| CFFT(512pt)       |     4          |          850                |         34633
 ^              | CFFT(1024pt)      |     13         |          1027               |         105388
 R5FSS1_0       | FIR(1KHz)         |     3          |          160                |         26942
 ^              | FIR(2KHz)         |     6          |          204                |         26981
 ^				| FIR(4KHz)         |     13         |          166                |         26965
 ^              | FIR(8KHz)         |     26         |          202                |         26994
 R5FSS1_1       | FOC(16KHz)        |     0          |          202                |         316
 ^              | FOC(32KHz)        |     1          |          164                |         318
 ^				| FOC(100KHz)       |     3          |          174                |         314
 ^              | FOC(250KHz)       |     10         |          199                |         315 
\endcond
