# EtherCAT SubDevice Setup with TwinCAT {#ETHERCAT_SUBDEVICE_DEMO_TWINCAT}

## Creating TwinCAT Project {#ETHERCAT_SUBDEVICE_DEMO_TWINCAT_PROJECT_CREATION}

- Install TwinCAT3. A 7 days free trial license is available for free download from the [Beckhoff's website](http://www.beckhoff.co.in/english.asp?download/tc3-download-xae.htm). Select the eXtended Automation Engineering (XAE) mode of installation.

- Copy the ESI file to `{TWINCAT_INSTALL_DIR}\TwinCAT\3.1\Config\Io\EtherCAT` folder.
    - ESI file for EtherCAT SubDevice application based on ETG stack is `${SDK_INSTALL_PATH}/source/industrial_comms/ethercat_slave/beckhoff_stack/esi/TI_ESC.xml`

- Start the TwinCAT XAE Shell application.

- Create a new TwinCAT XAE Project (XML format) by going to "File -> New -> Project -> TwinCAT Project"

    \image html EtherCAT_SubDevice_TwinCAT_Project_Create.PNG "Creating a new TwinCAT Project"

- Go to "TwinCAT -> Show Realtime Ethernet Compatible Devices" and install TwinCAT RT Ethernet intermediate driver. For best performance, it is recommended to use a compatible NIC card listed in [Supported network controllers](https://infosys.beckhoff.com/english.php?content=../content/1033/tc3_overview/9309844363.html&id=1489698440745036069) . For the first time only you need to do this for the PC's Ethernet port, which is used as the EtherCAT port. Go to "TwinCAT -> Show Realtime Ethernet Compatible Devices" and check if the ethernet adapter is listed below "Installed and ready to use devices" before attempting to scan the SubDevice

    <table style="border: 0 px; margin-left: auto; margin-right: auto">
        <tr>
            <td>
            \image html  EtherCAT_SubDevice_TwinCAT_NIC_Detection.PNG
            </td>
            <td>
            \image html  EtherCAT_SubDevice_TwinCAT_NIC_Detection_2.PNG
            </td>
        </tr>
    </table>

\note If your computer go to sleep and after awake it TwinCAT lost connection with SubDevice(s) and/or is not able to discover EtherCAT IO devices, you can fix this issue by going in TwinCAT to your Ethernet adapters, and disabling / enabling your NIC.

- Connect Ethernet cable from PC running TwinCAT to EtherCAT IN/Port0 of the PRU-ICSS on the EVM. If you have multiple devices in chain, please connect from EtherCAT OUT/Port1 to IN/Port0 of next device. For the last device in chain, OUT/Port1 will be left open (if NOT using redundancy mode).
\cond SOC_AM64X
    \image html  ethercat_SubDevice_ports.png
\endcond

\cond SOC_AM243X
<table style="border: 0 px; margin-left: auto; margin-right: auto">
    <tr>
        <td> \image html  ethercat_SubDevice_ports.png "AM243X-EVM"
        <td> \image html  lp_ethercat_SubDevice_ports.png "AM243X-LP"
    </tr>
</table>
\endcond

## Scanning the SubDevice {#ETHERCAT_SUBDEVICE_DEMO_TWINCAT_SCANNING_SUBDEVICE}

\cond SOC_AM64X
- On powering up the EVM and loading the application, you should see digital output LEDs 2, 4, 6 and 7 on. This indicates that SubDevice is up and in INIT state.
\endcond

\cond SOC_AM243X
- On powering up the AM243x-EVM and loading the application, you should see digital output LEDs 2, 4, 6 and 7 on. This indicates that SubDevice is up and in INIT state.
- On powering up the AM243x-LP and loading the application, you should see digital output LEDs 2 and 4 on. This indicates that SubDevice is up and in INIT state.
\endcond

- In Solution Explorer, go to "TwinCAT project -> I/O -> Devices". Right click on Devices and select Scan. Press OK in the next dialog to start scanning for EtherCAT devices.

    \image html EtherCAT_SubDevice_TwinCAT_SubDevice_Detection.PNG

- Once an EtherCAT compatible device has been detected on this Ethernet port, the following dialog shows up. Note that there is a tick mark next to the adapter to which the EVM is connected. Press OK and confirm to start "Scan for boxes".

    \image html EtherCAT_SubDevice_TwinCAT_SubDevice_Detection_2.PNG

- TI Box n(TIESC) will be detected automatically. The TI device will be listed "Box n (TIESC-*)". Press Yes to activate Free Run. This will put TI ESC into OP mode. TIESC number is based on the platform and application. Please check the ESI file for more details.

    \image html EtherCAT_SubDevice_TwinCAT_SubDevice_Detection_3.PNG

- The EtherCAT device state can be displayed by selecting a "Device" (Double click on "Device") and then selecting the "Online" tab. The device should be in the OP state with no Lost Frames or Tx/Rx Errors.

    \image html EtherCAT_SubDevice_TwinCAT_SubDevice_Detection_4.PNG

- The user can control 8 digital out LEDs using TwinCAT. LED\[0\] to LED\[7\] in "Box n (TIESC-*) -> DO Outputs -> LED". Each LED can be turned on and off by selecting "Online write 1" and "Online write 0" respectively.

    \image html EtherCAT_SubDevice_TwinCAT_SubDevice_Detection_5.PNG

\cond SOC_AM263X
\note On @VAR_BOARD_NAME_LOWER, LED control is not supported right now.
\endcond

- Above steps test the SubDevice in Free Run Operation Mode. For testing Distributed Clock (DC) Synchronization mode, refer to the \ref ETHERCAT_SUBDEVICE_DEMO_TWINCAT_DC_MODE_TESTING section.

## Testing DC Synchronization mode {#ETHERCAT_SUBDEVICE_DEMO_TWINCAT_DC_MODE_TESTING}

- Launch TwinCAT XAE Shell application and discover all the SubDevices in SubDevice chain. Multiple SubDevices are required to make use of distributed clocks. Here, we have used two devices in the chain.

- Choose DC Synchronization for all devices in the chain. Click on Box n (TIESC-*). Open the DC tab, and choose DC-Synchron option in Operation Mode. Do this for all SubDevices in the chain.

    \image html EtherCAT_SubDevice_DC_Mode.PNG

- In Solution Explorer, go to TwinCAT project, select "SYSTEM -> Real-Time". In the Settings tab, choose the desired Base Time. With compatible NIC cards, a base time of as low as 50 us can be chosen.

    \image html EtherCAT_SubDevice_DC_Mode_2.PNG

- Right-click on "SYSTEM -> Tasks". Click "Add New Item" on the pop-up menu. Select "TwinCAT Image with Task" in the next dialog box. Type a name for the task, and choose OK.

    <table style="border: 0 px; margin-left: auto; margin-right: auto">
        <tr>
            <td>
            \image html EtherCAT_SubDevice_DC_Mode_3.PNG
            </td>
            <td>
            \image html EtherCAT_SubDevice_DC_Mode_4.PNG
            </td>
        </tr>
    </table>

- In the "Task" tab for the created task, select Auto Priority Management. Press OK in dialog box saying "Global Priority Management should be turned on".

- Choose "Auto start" for the task. Choose 2 cycle ticks’ frequency for the task.

    \image html EtherCAT_SubDevice_DC_Mode_5.PNG

- Add an output variable to the task. Right-click on "[Task name] -> Outputs", and choose "Add New Item". You may choose default options for the variable.

    \image html EtherCAT_SubDevice_DC_Mode_6.PNG

- Link the variable to an output for the last SubDevice in the chain. Right-click on [Variable name], and choose "Change Link...".

    \image html EtherCAT_SubDevice_DC_Mode_7.PNG

- Choose an output from the last SubDevice. You may need to choose "All Types" in "Show Variable Types".

    \image html EtherCAT_SubDevice_DC_Mode_8.PNG

- You can launch into Run mode by clicking the  ![](EtherCAT_SubDevice_DC_Mode_9.PNG) "Activate Configuration" button on the toolbar.

- Choose Yes to generate mapping after modifying configuration, click on OK to "Activate Configuration?", and Yes to Restart in RUN mode.

## Testing Cable Redundancy {#ETHERCAT_SUBDEVICE_DEMO_TWINCAT_CABLE_REDUNDANCY}

Cable redundancy is a feature which ensures that all SubDevices are accessible even if there is a single link break in the network between two ports of MainDevice. We need to ports of MainDevice for testing this feature.

It can be tested as follows:

- Launch TwinCAT XAE Shell application and discover all the SubDevices in SubDevice chain. Here, we used two devices in the chain. The connections should be made as below:
    - MainDevice Port 0 to IN/Port0 of first device
    - OUT/Port1 of first device to IN/Port0 of second device
    - OUT/Port1 of second device to MainDevice Port 1

- The topology can be verified by selecting a "Device" (Double click on "Device") and then selecting the "EtherCAT" tab and click on "Topology". It is also possible to use Beckhoff CU2508 (Real-time Ethernet port multiplier) if you do not have two ports available on MainDevice. You can connect CU2508 to PC running TwinCAT, and use two ports of CU2508 to connect to two EtherCAT SubDevice devices.

  In our case, we have used CU2508 , therefore you see three devices (one CU2508 and two SubDevice devices) between MainDevice ports in the image below.

    \image html EtherCAT_SubDevice_Redundancy_Topology.PNG

- Configure the redundancy mode in TwinCAT. Available options are described in [Redundancy Mode](https://infosys.beckhoff.com/english.php?content=../content/1033/tc3_io_intro/1446583307.html&id=8240898519565691463) section of TwinCAT 3 documentation. Please refer to [Setting up: Device EtherCAT with cable redundancy](https://infosys.beckhoff.com/english.php?content=../content/1033/cu2508/2129825163.html&id=) for details on redundancy mode options to be used with CU2508.

- Activate DC mode as shown in \ref ETHERCAT_SUBDEVICE_DEMO_TWINCAT_DC_MODE_TESTING.

- Ensure that both devices are in OP state. It can be displayed by selecting the "Device" (Double click on "Device") and then selecting the "Online" tab.

    \image html EtherCAT_SubDevice_Redundancy_Before_link_break.PNG

- Break the link between OUT/Port1 of first device and IN/Port0 of second device. Both devices should still be in OP state.

    \image html EtherCAT_SubDevice_Redundancy_After_link_break.PNG

\note For this feature to work, `enhancedlink_enable` of \ref bsp_params should be set to \ref TIESC_MDIO_RX_LINK_ENABLE while initializing the EtherCAT SubDevice FWHAL. This configuration is done in `tiesc_socParamsInit()` in tiescsoc.c file.

## Generating ESI Header file From ESI XML file

[EtherCAT SubDevice Beckhoff SSC Example](\ref EXAMPLES_INDUSTRIAL_COMMS_ETHERCAT_SLAVE_BECKHOFF_SSC_DEMO) is expected to be used with `${SDK_INSTALL_PATH}/source/industrial_comms/ethercat_slave/beckhoff_stack/esi/TI_ESC.xml` file. If the application is modified to work with a different ESI XML file, user will need to update the corresponding ESI header file (tiesc_eeprom.h) and rebuild the application project.

- Generate the binary file equivalent to ESI XML file:
    - Configure TwinCAT as mentioned in \ref ETHERCAT_SUBDEVICE_DEMO_TWINCAT_PROJECT_CREATION and \ref ETHERCAT_SUBDEVICE_DEMO_TWINCAT_SCANNING_SUBDEVICE.
    - Click on the TI Box. Select "EtherCAT" Tab. Click on the "Advanced Settings" button.
    - Select "ESC Access -> E2PROM -> Hex Editor". Select "Write to File" and save the binary as a ".bin" file.

\note Please make sure that "Upload" button is not clicked any time during this step. It will load the EEPROM data from TI ESC to TwinCAT memory.

- Convert the binary file to header file using the bin2c tool. This tool can be found in `${SDK_INSTALL_PATH}/tools/bin2c` folder. Example usage is shown below.
  \code
  python ${SDK_INSTALL_PATH}/tools/bin2c/bin2c.py "Box 1 (TIESC-004).bin" new_tiesc_eeprom.c new_tiesc_eeprom
  \endcode

\cond SOC_AM64X

- Replace the contents within `"const unsigned char tiesc_eeprom[]"` array in `${SDK_INSTALL_PATH}/examples/industrial_comms/ethercat_slave_beckhoff_ssc_demo/@VAR_BOARD_NAME_LOWER/tiesc_eeprom.h` file with the data generated in `new_tiesc_eeprom.h` file within the brackets following `"#define NEW_TIESC_EEPROM"`.

\endcond

\cond SOC_AM243X

- For AM243X-EVM, replace the contents within `"const unsigned char tiesc_eeprom[]"` array in `${SDK_INSTALL_PATH}/examples/industrial_comms/ethercat_slave_beckhoff_ssc_demo/@VAR_BOARD_NAME_LOWER/tiesc_eeprom.h` file with the data generated in `new_tiesc_eeprom.h` file within the brackets following `"#define NEW_TIESC_EEPROM"`.

- For AM243X-LP, replace the contents within `"const unsigned char tiesc_eeprom[]"` array in `${SDK_INSTALL_PATH}/examples/industrial_comms/ethercat_slave_beckhoff_ssc_demo/@VAR_LP_BOARD_NAME_LOWER/tiesc_eeprom.h` file with the data generated in `new_tiesc_eeprom.h` file within the brackets following `"#define NEW_TIESC_EEPROM"`.

\endcond

\note Please do not directly replace the existing `tiesc_eeprom.h` file with new header file.

- Rebuild the application.

\cond SOC_AM64X || SOC_AM243X

## Online Application Upgrade

\note Application should be loaded with \ref EXAMPLES_DRIVERS_SBL_OSPI method to test this feature using following steps

- Configure TwinCAT as mentioned in \ref ETHERCAT_SUBDEVICE_DEMO_TWINCAT_PROJECT_CREATION and \ref ETHERCAT_SUBDEVICE_DEMO_TWINCAT_SCANNING_SUBDEVICE.

- Click on TI Box. Select "Online" tab.

- Click "Bootstrap" button. This will take the SubDevice to BOOT state.

    \image html EtherCAT_SubDevice_FOE_1.PNG

- Once the state has changed to "BOOT", Click "Download" button.

    \image html EtherCAT_SubDevice_FOE_2.PNG

- Rename your EtherCAT application binary (.appimage file) as `ECATFW__`, and use this file as your new EtherCAT application.

    \image html EtherCAT_SubDevice_FOE_3.PNG

- Locate the new application to be dowloaded click "Open".

- Click OK on the new dialog shown.

    \image html EtherCAT_SubDevice_FOE_4.PNG

- This will download the new application. The progress bar will show the status of download.

- Once the download has finished, change the state back to "Init" by clicking "Init" button.

- Perform a warm reset by writing "RST"(0x545352) or "rst"(0x747372) to ESC Reset(0x0E14-0x0E17) register to reload the application. Power restart of the EVM will also load the new application.

    \image html EtherCAT_SubDevice_FOE_5.PNG

\endcond
