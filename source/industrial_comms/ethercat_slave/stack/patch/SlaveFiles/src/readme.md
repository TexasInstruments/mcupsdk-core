Steps to patch and rebuild the EtherCAT Slave Stack Code (SSC) library for "examples/industrial_comms/ethercat_slave_demo" examples
------------------------------------------------------------------------------------------------------------------------------

Examples in "examples/industrial_comms/ethercat_slave_demo" use an evaluation version of the stack. They will run for 1 hour only. If you want an unlimited version, you need to rebuild the Beckhoff SSC Library used by the examples with a patched version of the original SSC code.

(1) To build the custom SSC library, it is necessary to get the EtherCAT Slave Stack Code (SSC) sources

    - Download the EtherCAT slave sample code version 5.13 from ETG website (http://www.ethercat.org/).
    - unzip the downloaded SSC_V5i13.zip into a temporary directory SSC_V5i13
    - enter SSC_V5i13 and unzip SlaveFiles.zip into SlaveFiles
    - copy SlaveFiles/src/* to ${SDK_INSTALL_PATH}/source/industrial_comms/ethercat_slave/stack/patch/SlaveFiles/src

(2) Download additional tools

Windows
    - Windows Patch Utility from gnuwin32 sourceforge(http://gnuwin32.sourceforge.net/downlinks/patch-bin-zip.php). (Note that this is not a TI tool. See licensing information(http://savannah.gnu.org/projects/patch/) page for more details)
    - Download unix2dos-Text file format converters from gnuwin32 sourceforge(https://sourceforge.net/projects/dos2unix/). (Note that this is not a TI tool. See licensing information(http://www.freebsd.org/copyright/freebsd-license.html) page for more details)
    - patch.exe, dos2unix.exe, unix2dos.exe utilities can be found in their respective bin folders.
    - Copy those binaries to ${SDK_INSTALL_PATH}/source/industrial_comms/ethercat_slave/stack/patch

Linux
    - install dos2unix and patch from your package manager

(3) Build the library using following step

    - change directory to ${SDK_INSTALL_PATH}/source/industrial_comms/ethercat_slave/stack/patch
    - $(MAKE) -s -f makefile.custom-ssc ssc
    - the resulting custom SSC libraries are in ${SDK_INSTALL_PATH}/source/industrial_comms/ethercat_slave/stack

(4) Rebuild the example with custom SSC
