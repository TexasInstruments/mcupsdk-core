#  Getting Started {#GETTING_STARTED}

\attention It is strongly recommended to begin development by following the steps in the order listed below.

\note The steps in this user guide show screen shots and descriptions based on Windows.
      However the steps in Linux would remain the same, unless mentioned otherwise.

\note ${SDK_INSTALL_PATH} refers to the path where the SDK is installed. Recommend to install in
      `C:/ti` in Windows, and `${HOME}/ti` in Linux. To install to other locations, refer \ref INSTALL_NON_DEFAULT

\note `{some text}` refers to a variable string that should be replaced by user and `{some text}` should not be typed verbatim
      on the command prompt.

\note In Windows, use the `cmd.exe` as command prompt and in Linux, use the bash shell as the command prompt.

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
\note For performance measurements, use compiler optimization level -Os. Refer developer guide \ref PERFORMANCE_OPTIMIZATIONS_GUIDE for more details
\endcond

## Introduction

### Getting Started Goals

On successful completion of below steps, you would have achieved the following
- All tools needed for development are installed
\if SOC_AM65X
- IDK setup needed for development is verified
\else
- EVM setup needed for development is verified
\endif
- CCS IDE setup needed for development is verified
- One SDK example has been built using makefile as well as CCS project
\if SOC_AM65X
- One SDK example has been loaded and run on the IDK
\else
- One SDK example has been loaded and run on the EVM
\endif
- Output console logs on CCS and UART are working as expected
\if !SOC_AM65X
- One SDK example has been flashed to the EVM flash and the application booted from the flash without CCS.
\endif
\cond SOC_AM263PX || SOC_AM243X || SOC_AM263X || SOC_AM261X
- Use \htmllink{https://www.ti.com/lit/an/spradf0a/spradf0a.pdf, TI OptiFlah memory technology} along with [Smart Placement](\ref SMART_PLACEMENT_GETTING_STARTED) tool to improve system level performance with external flash.
\endcond

### Terms and Abbreviations

\cond SOC_AM243X || SOC_AM64X
Term                    | Description
------------------------|------------
McSPI Controller        | McSPI in communication which generates the clock and initiates the transfer
McSPI Peripheral        | McSPI in communication which read and write data based on Controller clock
I2C Controller          | I2C device which generates the SCL clock and initiates communication
I2C Target              | I2C device which responds to I2C Controller
LIN Commander           | LIN device which intiates the transfer
LIN Responder           | LIN device which responds to the transfer
PTP Sync Time Source    | Ethernet device that periodically broadcasts the current time as a message to the other clocks
PTP Sync Receiver       | Ethernet device that is waiting for the master to poll for data to actually send it
EtherCAT MainDevice     | EtherCAT device which is allowed to trasfer the data across network
EtherCAT SubDevice      | EtherCAT device which reads the data addressed to them bu MainDevice
Ethernet/IP Scanner     | Ethernet/IP scanned is the client device responsible for establishing a connection path to an EtherNet/IP Adapter
Ethernet/IP Adapter     | Ethernet/IP adapter which receives implicit communication connection requests from an scanner then produces its I/O data at the requested rate. An adapter is also an explicit message server.
IO-Link Controller      | IO-Link device which provides the interface to the higher-level controller (PLC) and controls the communication with the connected IO-Link devices. An IO-Link master can have one or more IO-Link ports to which only one device can be connected at a time
IO-Link Device          | IO-Link device is a short distance, bi-directional, digital, point-to-point, wired (or wireless), industrial communications networking standard (IEC 61131-9) used for connecting digital sensors and actuators to either a type of industrial fieldbus or a type of industrial Ethernet
\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
Term                    | Description
------------------------|------------
McSPI Controller        | McSPI in communication which generates the clock and initiates the transfer
McSPI Peripheral        | McSPI in communication which read and write data based on Controller clock
I2C Controller          | I2C device generates the SCL clock and initiates communication
I2C Target              | I2C device which responds to I2C Controller
LIN Commander           | LIN device which intiates the transfer
LIN Responder           | LIN device which responds to the transfer
PTP Sync Time Source    | Ethernet device that periodically broadcasts the current time as a message to the other clocks
PTP Sync Receiver       | Ethernet device that is waiting for the master to poll for data to actually send it
EtherCAT MainDevice     | EtherCAT device which is allowed to trasfer the data across network
EtherCAT SubDevice      | EtherCAT device which reads the data addressed to them bu MainDevice
\endcond

\cond SOC_AM273X || SOC_AWR294X
Term                    | Description
------------------------|------------
MibSPI Controller       | MibSPI in communication which generates the clock and initiates the transfer
MibSPI Peripheral       | MibSPI in communication which read and write data based on Controller clock
I2C Controller          | I2C device generates the SCL clock and initiates communication
I2C Target              | I2C device which responds to I2C Controller
LIN Commander           | LIN device which intiates the transfer
LIN Responder           | LIN device which responds to the transfer
PTP Sync Time Source    | Ethernet device that periodically broadcasts the current time as a message to the other clocks
PTP Sync Receiver       | Ethernet device that is waiting for the master to poll for data to actually send it
\endcond

\cond SOC_AM65X
Term                    | Description
------------------------|------------
I2C Controller          | I2C device generates the SCL clock and initiates communication
I2C Target              | I2C device which responds to I2C Controller
\endcond

### Getting Started Steps

- **Step 1:** Download, install SDK and related tools, see \subpage SDK_DOWNLOAD_PAGE [**NEEDS TO BE DONE ONCE when SDK is installed**]

- **Step 2:** Download, install and setup CCS for development, see \subpage CCS_SETUP_PAGE [**NEEDS TO BE DONE ONCE when SDK is installed**]

\if SOC_AM65X
- **Step 3:** Setup IDK for program execution, see \subpage IDK_SETUP_PAGE
- **Step 4:** Build a "hello world" example for the IDK, see \subpage GETTING_STARTED_BUILD
- **Step 5:** Load and run the "hello world" example on the IDK, see \subpage CCS_LAUNCH_PAGE
\else
- **Step 3:** Setup EVM for program execution, see \subpage EVM_SETUP_PAGE

\cond SOC_AM273X || SOC_AM263X || SOC_AM263PX || SOC_AM261X
- **Step 4:** Loading and running the example, see \subpage LOAD_RUN_EXAMPLE

- **Step 5:** Additional Details, see \subpage ADDITIONAL_DETAILS_PAGE
\endcond

\cond SOC_AWR294X || SOC_AM243X || SOC_AM64X
- **Step 4:** Build a "hello world" example for the EVM, see \subpage GETTING_STARTED_BUILD

- **Step 5:** Load and run the "hello world" example on the EVM, see \subpage CCS_LAUNCH_PAGE

\cond  SOC_AM243X || SOC_AM64X

- **Step 6:** Flash the "hello world" example on the EVM and boot without CCS, see \subpage GETTING_STARTED_FLASH or \subpage GETTING_STARTED_FLASH_DFU [**OPTIONAL and is typically needed after all development via CCS is done**]

\endcond

\cond SOC_AWR294X || SOC_AM273X || SOC_AM263X || SOC_AM263PX || SOC_AM261X
- **Step 6:** Flash the "hello world" example on the EVM and boot without CCS, see \subpage GETTING_STARTED_FLASH [**OPTIONAL and is typically needed after all development via CCS is done**]
\endcond

\endcond

\cond SOC_AM62X
- **Step 4:** Build a "hello world" example for the EVM, see \subpage GETTING_STARTED_BUILD

- **Step 5:** Load and run the "hello world" example on the EVM, see \subpage CCS_LAUNCH_PAGE

\endcond

\cond  SOC_AM263X || SOC_AM263PX || SOC_AM243X || SOC_AM261X
- **Step 7:** Optimized system level performance using Smart Placement. See \subpage SMART_PLACEMENT_GETTING_STARTED.
\endcond
\endif
### Next Steps

Now you can explore the SDK by running more examples (see \ref EXAMPLES) and browsing through various developer notes (see \ref DEVELOPER_GUIDES), to understand the SDK better and develop your own applications with the SDK.
