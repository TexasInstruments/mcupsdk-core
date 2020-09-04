# EtherCAT SubDevice Beckhoff SSC Demo{#EXAMPLES_INDUSTRIAL_COMMS_ETHERCAT_SLAVE_BECKHOFF_SSC_DEMO}

[TOC]

# Introduction

This example is a EtherCAT SubDevice application based on EtherCAT SubDevice Stack Code (SSC) version 5.13 from Beckhoff.

The example does the below
- Initializes the EtherCAT SubDevice stack
- Initializes the FWHAL, which kicks off the ESC
- SubDevice is taken to INIT state. It can be move to OP (operational) state by connecting it to a EtherCAT MainDevice, and process data communication is possible after that.
\cond SOC_AM64X
- Allows user to control 8 LEDs using an EtherCAT MainDevice
\endcond
\cond SOC_AM243X
- Allows user to control 8 LEDs for AM243X-EVM and 4 LEDs for AM243X-LP using an EtherCAT MainDevice
\endcond
\cond SOC_AM64X || SOC_AM243X
- Allows Online Application Upgrade, if loaded using SBL OSPI
\endcond

\cond SOC_AM243X

\note

 In the E1/E2 revision of @VAR_LP_BOARD_NAME_LOWER board, there are following issues which impact enhanced link detection and RX_ER monitor by ICSSG. Therefore EtherCAT cable redundancy can not be supported in E1/E2 revision.
 - "PRG1_PRU0_GPO8" is not connected to "PRG1_CPSW_ETH1_LED_LINK" from Ethernet PHY
 - "PRG1_PRU1_GPO8" is not connected to "PRG1_CPSW_ETH2_LED_LINK" from Ethernet PHY
 - "PRG1_PRU0_GPO5" is not connected to "PRG1_CPSW_ETH1_LED_1000/RX_ER" from Ethernet PHY
 - "PRG1_PRU1_GPO5" is not connected to "PRG1_CPSW_ETH2_LED_1000/RX_ER" from Ethernet PHY

 In E3 revision, following connections are available. SDK example enables cable redundancy and will work on E3 revision only. For running it on E1/E2 revision, few changes are required. For more details, see \ref EXAMPLES_INDUSTRIAL_COMMS_ETHERCAT_SLAVE_BECKHOFF_SSC_DEMO_COMBOS.

\endcond

\cond SOC_AM64X || SOC_AM243X

\note The work-around for issue "i2329 - MDIO: MDIO interface corruption (CPSW and PRU-ICSS)" (described in <a href="https://www.ti.com/lit/er/sprz457e/sprz457e.pdf">AM64x/AM243x Processor Silicon Revision 1.0, 2.0 (Rev. E)</a>) is available in this release. Please see \ref INDUSTRIAL_COMMS_MDIO_MANUALMODE_FW_USAGE for more details.

\endcond

\cond SOC_AM263X

\note
Following features are not tested or implemented in this release :
- Use of flash (Online Application Upgrade) is not verified
- Run and Error LEDs are not implemented
- LEDs linked to Process Data are not implemented
- Use of I2C based EEPROM memory for TIESC EEPROM is not implemented
\endcond

# Supported Combinations {#EXAMPLES_INDUSTRIAL_COMMS_ETHERCAT_SLAVE_BECKHOFF_SSC_DEMO_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ICSSG          | ICSSG1
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/industrial_comms/ethercat_SubDevice_beckhoff_ssc_demo

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ICSSG          | ICSSG1
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER (E3 Revision)
 Example folder | examples/industrial_comms/ethercat_SubDevice_beckhoff_ssc_demo

As mentioned above, SDK example will work on E3 revision of @VAR_LP_BOARD_NAME_LOWER only. For running it on E1/E2 revision, following changes are needed.

- Disable enhanced link detection
    - In `tiesc_socParamsInit()` function present in "${SDK_INSTALL_PATH}/examples/industrial_comms/ethercat_SubDevice_beckhoff_ssc_demo/am243x-lp/tiescsoc.c", set `bspInitParams->enhancedlink_enable` to `TIESC_MDIO_RX_LINK_DISABLE`.

- Disable the pinmux configuration for pins not available in E1/E2 revision
    - In "EtherCAT" module in SysConfig, uncheck following pins from PRU_ICSSG1_MII_G_RT.
        - MII0_RXER(PR1_MII0_RXER)
        - MII0_RXLINK(PR1_MII0_RXLINK)
        - MII1_RXER(PR1_MII1_RXER)
        - MII1_RXLINK(PR1_MII1_RXLINK)


\endcond

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/industrial_comms/ethercat_SubDevice_beckhoff_ssc_demo

\endcond

\cond SOC_AM64X || SOC_AM243X

# Performance Benchmarking

TwinCAT MainDevice was used as the EtherCAT MainDevice for these tests.

## Cycle Time
Lowest cycle time tested is **31.25 us(microseconds)** with Distributed Clock(DC) Synchronization mode.

## Interrupt Processing Time

Following is the interrupt processing time for PDI and Sync ISRs with 50 us cycle time. The RxPDO size is 5 bytes and TxPDO size is 7 bytes in this example.

<table>
<tr>
    <th>Scenario
    <th>PDI ISR Processing Time (microseconds)
    <th>SYNC0 ISR Processing Time (microseconds)
    <th>SYNC1 ISR Processing Time (microseconds)
</tr>
<tr>
    <td> DC mode with SYNC0 enabled
    <td> 2.7
    <td> 3.2
    <td> 0
</tr>
<tr>
    <td> DC mode with SYNC0 and SYNC1 enabled
    <td> 2.7
    <td> 0.8
    <td> 2.6
</tr>
</table>

\endcond

# Steps to Run the Example

- To build this example, it is necessary to get the EtherCAT SubDevice Stack Code (SSC). Download EtherCAT stack version 5.13 from [ETG website](http://www.ethercat.org/) and extract it to a local folder. Please refer to "Application Note ET9300 (EtherCAT SubDevice Stack Code)" for more details on SSC.
- Generate the patched EtherCAT SubDevice stack code source files using any one of the below mentioned methods:
    - **Using the patch file**
        - Copy the EtherCAT SubDevice Stack files to `{SDK_INSTALL_PATH}/source/industrial_comms/ethercat_SubDevice/beckhoff_stack/stack_sources/` folder.
        - Download Windows Patch Utility from [gnuwin32 sourceforge](http://gnuwin32.sourceforge.net/downlinks/patch-bin-zip.php). (Note that this is not a TI tool. See [licensing information](http://savannah.gnu.org/projects/patch/) page for more details)
        - Download Dos2Unix/Unix2Dos-Text file format converters from [gnuwin32 sourceforge](https://sourceforge.net/projects/dos2unix/). (Note that this is not a TI tool. See [licensing information](http://www.freebsd.org/copyright/freebsd-license.html) page for more details)
        - Patch file utility(Patch.exe) and unix2dos.exe utility can be found in their bin folders.
        - Launch DOS Command prompt
        - CD to the folder `${SDK_INSTALL_PATH}/source/industrial_comms/ethercat_SubDevice/beckhoff_stack/patch` which contains TI_ECAT.patch.
        - Execute unix2dos.exe as given below:
          \code
          $(Dos2Unix/Unix2Dos-DIR)/bin/unix2dos.exe TI_ECAT.patch
          \endcode
        - CD to patch file utility bin folder.
        - Execute patch.exe as given below:
          \code
          patch.exe -i ${SDK_INSTALL_PATH}/source/industrial_comms/ethercat_SubDevice/beckhoff_stack/patch/TI_ECAT.patch -d ${SDK_INSTALL_PATH}/source/industrial_comms/ethercat_SubDevice/beckhoff_stack/stack_sources/
          \endcode
    - **Using Beckhoff SSC Tool**
        - Install SSC tool version 1.5.3.0. This configuration tool facilitates working with the EtherCAT SubDevice Stack Code (SSC), as it allows reducing the size of the EtherCAT SubDevice stack code by removing unused code parts depending on the desired configuration. Objects should be defined in a .xlsx file. Please refer to [EtherCAT SubDevice Design - Quick Guide](https://download.beckhoff.com/download/document/io/ethercat-development-products/ethercat_SubDevice_design_quick_guide.pdf) for details.
        - Click on "Import" button and select TI_ESC_[SDK_VERSION].xml present in SoC specific folder inside `${SDK_INSTALL_PATH}/source/industrial_comms/ethercat_SubDevice/beckhoff_stack/patch` folder.
        - Make sure "Custom" is selected in the dialog box and "TI [SOC] Sample \<Texas Instruments Incorporated\>" is selected from the list.
        - Set DC_SUPPORTED to 1 if not set.
        - Save the project.
        - Click "Project->Create new SubDevice Files". This will generate the EtherCAT Source files specific to the the selected TI device.
        - Copy all the generated files except tiescappl.c, tiescappl.h and tiescapplObjects.h to `${SDK_INSTALL_PATH}/source/industrial_comms/ethercat_SubDevice/beckhoff_stack/stack_sources/`.

        \note
        - For the SDK example, tiescappl.c and tiescappl.h files are used from "${SDK_INSTALL_PATH}/examples/industrial_comms/ethercat_SubDevice_beckhoff_ssc_demo/" folder, and not from the SSC Tool generated files.
        - If you want to modify the object dictionary, you can update the "${SDK_INSTALL_PATH}/source/industrial_comms/ethercat_SubDevice/beckhoff_stack/patch/am64x_am243x_am263x/tiescappl.xlsx" file and then the SSC tool will generate the application code accordingly.

- Change macro defintions in `{SDK_INSTALL_PATH}/source/industrial_comms/ethercat_SubDevice/beckhoff_stack/stack_sources/ecat_def.h`, if required for your application. Please ensure that TIESC_HW is set to 1, and TIESC_APPLICATION is set to 1.
- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- To test the application, one can use TwinCAT or any other compatible EtherCAT MainDevice. The steps to test the SubDevice with TwinCAT are present in \subpage ETHERCAT_SUBDEVICE_DEMO_TWINCAT.

\attention If you need to reload and run the example again, EVM power-cycle is MUST.

# Sample Output

Shown below is a sample output when the application is run:

\code
EtherCAT Device
EtherCAT Sample application
Revision/Type : x0590 Build : x04FD
Firmware Version : 5.4.253
SYNC0 task started
SYNC1 task started
\endcode

# See Also

\ref ETHERCAT_SUBDEVICE_FWHAL
