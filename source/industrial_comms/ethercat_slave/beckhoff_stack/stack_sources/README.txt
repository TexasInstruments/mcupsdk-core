Steps to generate patched Slave Stack Code
-------------------------------------------

(1) To build the "{SDK_INSTALL_PATH}/examples/industrial_comms/ethercat_slave_beckhoff_ssc_demo" example, it is necessary to get the EtherCAT Slave Stack Code (SSC). Download EtherCAT stack version 5.12 from ETG website(http://www.ethercat.org/) and extract it to a local folder.

(2) Generate the patched EtherCAT Slave stack code source files using any one of the below mentioned methods:

    (a) Using the patch file
        - Copy the EtherCAT Slave Stack files to "{SDK_INSTALL_PATH}/source/industrial_comms/ethercat_slave/beckhoff_stack/stack_sources/" folder.
        - Download Windows Patch Utility from gnuwin32 sourceforge(http://gnuwin32.sourceforge.net/downlinks/patch-bin-zip.php). (Note that this is not a TI tool. See licensing information(http://savannah.gnu.org/projects/patch/) page for more details)
        - Download Dos2Unix/Unix2Dos-Text file format converters from gnuwin32 sourceforge(https://sourceforge.net/projects/dos2unix/). (Note that this is not a TI tool. See licensing information(http://www.freebsd.org/copyright/freebsd-license.html) page for more details)
        - Patch file utility(Patch.exe) and unix2dos.exe utility can be found in their bin folders.
        - Launch DOS Command prompt
        - CD to the folder "${SDK_INSTALL_PATH}/source/industrial_comms/ethercat_slave/beckhoff_stack/patch" which contains TI_ECAT.patch.
        - Execute unix2dos.exe as given below:
          $(Dos2Unix/Unix2Dos-DIR)/bin/unix2dos.exe TI_ECAT.patch
        - CD to patch file utility bin folder.
        - Execute patch.exe as given below:
          patch.exe -i ${SDK_INSTALL_PATH}/source/industrial_comms/ethercat_slave/beckhoff_stack/patch/TI_ECAT.patch -d ${SDK_INSTALL_PATH}/source/industrial_comms/ethercat_slave/beckhoff_stack/stack_sources/

    (b) Using Beckhoff SSC Tool
        - Install SSC tool version 1.4.0.0. This configuration tool facilitates working with the EtherCAT Slave Stack Code (SSC), as it allows reducing the size of the EtherCAT slave stack code by removing unused code parts depending on the desired configuration. Objects should be defined in a .xlsx file. Please refer to "EtherCAT Slave Design - Quick Guide"(https://download.beckhoff.com/download/document/io/ethercat-development-products/ethercat_slave_design_quick_guide.pdf) for details.
        - Click on "Import" button and select TI_ESC_[SDK_VERSION].xml present in SoC specific folder inside "${SDK_INSTALL_PATH}/source/industrial_comms/ethercat_slave/beckhoff_stack/patch/" folder.
        - Make sure "Custom" is selected in the dialog box and "TI [SOC] Sample <Texas Instruments Incorporated>" is selected from the list.
        - Set DC_SUPPORTED to 1 if not set.
        - Save the project.
        - Click "Project->Create new Slave Files". This will generate the EtherCAT Source files specific to the the selected TI device.
        - Copy all the generated files except tiescappl.c, tiescappl.h and tiescapplObjects.h to `${SDK_INSTALL_PATH}/source/industrial_comms/ethercat_slave/beckhoff_stack/stack_sources/`.
        - NOTE :
            - For the SDK example, tiescappl.c and tiescappl.h files are used from "${SDK_INSTALL_PATH}/examples/industrial_comms/ethercat_slave_beckhoff_ssc_demo/" folder, and not from the SSC Tool generated files.
            - If you want to modify the object dictionary, you can update the "${SDK_INSTALL_PATH}/source/industrial_comms/ethercat_slave/beckhoff_stack/patch/am64x_am243x_am263x/tiescappl.xlsx" file and then the SSC tool will generate the application code accordingly.

NOTE : Please refer to the documentation of this example in SDK documentation for more details. Open "${SDK_INSTALL_PATH}/README_FIRST_*.html" and go to "Examples and Demos -> Networking -> EtherCAT Slave" page.