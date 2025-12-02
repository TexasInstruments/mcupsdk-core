
# SYSFW Tools {#TOOLS_SYSFW}

[TOC]

\note To see the exact sequence of steps in which boardcfg generation is done, see the `makefile` inside the ${SDK_INSTALL_PATH}/tools/sysfw/boardcfg/ folder.

## Introduction

This section describes the various tools used in conjunction with System Controller Firmware (SYSFW)

## Tool requirements on host PC

- The tools mentioned are implemented using python and needs python version 3.x
- Refer to the page, \ref INSTALL_PYTHON3 , to install python and the required python packages on your PC.
\cond SOC_AM243X || SOC_AM64X
- SysConfig tool for using K3 Resource Partitioning Tool: [Download SysConfig](https://www.ti.com/tool/download/SYSCONFIG)
\endcond

## Important files and folders

<table>
<tr>
    <th>Folder/Files
    <th>Description
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/tools/bin2c/</td></tr>
<tr>
    <td>bin2c.py
    <td>Tool to convert a binary file to a C array of hexadecimals
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/tools/sysfw/boardcfg/</td></tr>
<tr>
    <td>sysfw_boardcfg_validator.py
    <td>Python script which validates the boardcfg. Used internally in the boardcfg makefile
</tr>
</table>

\cond SOC_AM243X || SOC_AM64X
## K3 Resource Partitioning Tool {#K3_RESPART_TOOL}

This tool is based on Texas Instrument's SysConfig tool. It allows you to configure various system level
parameters and generate the data which can be fed into many software components.
Typical usage for this tool is for System integrators, where one would be  able
to partition **various resources** across different software components. These
resources includes DMA channels, rings, proxies, interrupts, etc. Apart from
this, the tool supports configuration of QoS (Quality of Service) and Firewall
parameters which helps in ensuring partitioning of **peripheral devices** across
different CPUs or virtual machines.

Please refer to \ref RESOURCE_ALLOCATION_GUIDE guide if you want to modify the default resources using K3 Resource Partitioning Tool.
\endcond

## SYSFW Board Config Generation {#BOARDCFG_GEN}

\if (SOC_AM243X || SOC_AM64X)
\note Please refer to \ref K3_RESPART_TOOL section if you want to modify the default resources.
\else
\note Please refer this \ref RESOURCE_ALLOCATION_GUIDE guide if you want to modify the default resources.
\endif

SYSFW Board Config is a SOC specific configuration data regarding the various system attributes controlled by the SYSFW. These include resources, power and clock, security etc. This configuration is sent to SYSFW during boot time. The default configuration is stored in `source/drivers/sciclient/sciclient_defaultBoardCfg/{SOC}/`

- Resource Management BoardCfg - sciclient_defaultBoardCfg_rm.c
- Power Management BoardCfg - sciclient_defaultBoardCfg_pm.c
- Security BoardCfg - sciclient_defaultBoardCfg_security.c

- For sending it to SYSFW, these files are converted to hex arrays. We use the bin2c.py python script to do this. This is done internally in the boardcfg makefile. If we change the boardcfg in the above mentioned files, run the following command to generate the hex array header files

\cond SOC_AM243X
```bash
make -s -C tools/sysfw/boardcfg sciclient_boardcfg SOC=am243x
```
\endcond
\cond SOC_AM64X
```bash
make -s -C tools/sysfw/boardcfg sciclient_boardcfg SOC=am64x
```
\endcond

- Once these header files are generated, rebuild the libraries by doing

\code
cd ${SDK_INSTALL_PATH}
gmake -s libs DEVICE={device}
\endcode
\cond !SOC_AM65X
- After this, make sure to rebuild the secondary bootloader (SBL) applications. You can do this by

\code
cd ${SDK_INSTALL_PATH}
gmake -s sbl DEVICE={device}
\endcode

- If you're not using any of the SBLs (SBL UART, SBL OSPI, SBL NULL) and are following the CCS boot method (\ref EVM_SOC_INIT_NOBOOT_MODE), make sure to build the sciclient_ccs_init application by doing

\code
cd ${SDK_INSTALL_PATH}
gmake -s -C examples/drivers/sciclient/sciclient_ccs_init/{board}/r5fss0-0_nortos/ti-arm-clang
\endcode

\note This step is only needed if you are using the CCS boot method

\cond SOC_AM64X
- Once the build is completed, copy the .out file generated and replace with the one already present in ${SDK_INSTALL_PATH}/tools/ccs_load/am64x/ folder.
\endcond
\cond SOC_AM243X
- Once the build is completed, copy the .out file generated and replace with the one already present in ${SDK_INSTALL_PATH}/tools/ccs_load/am243x/ folder.
\endcond
\endcond
\cond SOC_AM64X || SOC_AM243X

### SYSFW Trace Enable {#SYSFW_TRACE_ENABLE}

To enable SYSFW traces, change `#undef SYSFW_TRACE_ENABLE` to `#define SYSFW_TRACE_ENABLE` in `source/drivers/sciclient/sciclient_default_boardcfg/{SOC}/sciclient_defaultBoardcfg.c`. Then rebuild the boardcfg as explained in the above section. This enables **all** the following trace sources and destinations -

- **Trace Destinations** (`trace_dst_enables`): Controls where logs are output.
  - `TISCI_BOARDCFG_TRACE_DST_UART0` - MAIN UART output
  - `TISCI_BOARDCFG_TRACE_DST_MEM` - Memory buffer
  - `TISCI_BOARDCFG_TRACE_DST_ITM` - CCS Console

- **Trace Sources** (`trace_src_enables`): Controls which components generate logs
  - `TISCI_BOARDCFG_TRACE_SRC_PM` - Power Management traces
  - `TISCI_BOARDCFG_TRACE_SRC_RM` - Resource Management traces
  - `TISCI_BOARDCFG_TRACE_SRC_SEC` - Security traces
  - `TISCI_BOARDCFG_TRACE_SRC_BASE` - Baseport traces
  - `TISCI_BOARDCFG_TRACE_SRC_USER` - User-level traces
  - `TISCI_BOARDCFG_TRACE_SRC_SUPR` - Supervisor-level traces

#### Selective Trace Enable

To **selectively** enable specific traces instead of all, follow these steps:

1. **Locate board configuration file**:
   - Path: `source/drivers/sciclient/sciclient_default_boardcfg/{SOC}/sciclient_defaultBoardcfg.c`
   - Edit the `.debug_cfg` section in the `tisci_boardcfg` structure

2. **Configure trace settings** based on your needs:

   In the `#else` block (when `SYSFW_TRACE_ENABLE` is not defined), replace the default values with your desired configuration:

   **Example 1: Enable PM and RM logs to both UART and memory**
   \code{.c}
   .trace_dst_enables = TISCI_BOARDCFG_TRACE_DST_UART0 |
                        TISCI_BOARDCFG_TRACE_DST_MEM,
   .trace_src_enables = TISCI_BOARDCFG_TRACE_SRC_PM |
                        TISCI_BOARDCFG_TRACE_SRC_RM,
   \endcode

   **Example 2: Enable security logs to UART**
   \code{.c}
   .trace_dst_enables = TISCI_BOARDCFG_TRACE_DST_UART0,
   .trace_src_enables = TISCI_BOARDCFG_TRACE_SRC_SEC |
                        TISCI_BOARDCFG_TRACE_SRC_BASE,
   \endcode

3. **Rebuild and reflash board configuration**:
   - Follow the instructions in \ref BOARDCFG_GEN section to regenerate boardcfg
   - Reflash the SBL with the updated board configuration
   - The SYSFW will automatically load the new configuration at runtime

#### Viewing Trace Output

Depending on the destination flags enabled, logs can be viewed from different locations:

**UART Destination** (`TISCI_BOARDCFG_TRACE_DST_UART0`):

- SYSFW traces are output to **MAIN_UART1**
  - On Linux host machine, this typically appears as `/dev/ttyUSB2`
  - **Note**: The exact USB device number (`/dev/ttyUSBx`) may vary depending on your EVM configuration and USB connection order. Check `dmesg | grep tty` after connecting the EVM to identify the correct port.
  - On Windows host machine, use Device Manager to identify the COM port for the MAIN UART
- **Terminal settings**: 115200 baud, 8N1 (8 data bits, no parity, 1 stop bit)
- **Recommended tools**: minicom, putty, or any serial terminal

**Memory Destination** (`TISCI_BOARDCFG_TRACE_DST_MEM`):

- Logs stored in a memory buffer
- Memory buffer location: See \htmllink{https://software-dl.ti.com/tisci/esd/latest/4_trace/trace.html#trace-memory-buffer-location, Trace memory buffer documentation}

**ITM Destination** (`TISCI_BOARDCFG_TRACE_DST_ITM`):

- Logs appear in CCS Console when debugging with Code Composer Studio

## SYSFW Trace Parser {#SYSFW_TRACE_PARSER}

After collecting the SYSFW logs as described in the above section, parse them using the `sysfw_trace_parser.py` script. This will decode the hex trace values and produce a readable text file as output, which helps interpret the logs and debug system issues.

### Parsing Trace Logs

- Run the python script on the Windows command prompt (`cmd.exe`) or Linux bash shell:

\code
cd ${SDK_INSTALL_PATH}/tools/sysfw/trace_parser
python sysfw_trace_parser.py --log_file <path_to_log_file> --output_file <output_text_file>
\endcode

**Example:**
\code
python sysfw_trace_parser.py --log_file sysfw_uart_log.txt --output_file decoded_trace.txt
\endcode

### Getting Help

To view all available command-line options:

\code
python sysfw_trace_parser.py --help
\endcode

### Additional Resources

For more details on trace format and interpretation, refer to:
\htmllink{https://software-dl.ti.com/tisci/esd/latest/4_trace/trace.html, System Firmware Trace Layer Documentation}

\endcond
