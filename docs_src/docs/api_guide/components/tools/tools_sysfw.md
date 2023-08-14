
# SYSFW Tools {#TOOLS_SYSFW}

[TOC]

\note To see the exact sequence of steps in which boardcfg generation is done, see the `makefile` inside the ${SDK_INSTALL_PATH}/tools/sysfw/boardcfg/ folder.

## Introduction

This section describes the various tools used in conjunction with System Controller Firmware (SYSFW)

## Tool requirements on host PC

- The tools mentioned are implemented using python and needs python version 3.x
- Refer to the page, \ref INSTALL_PYTHON3 , to install python and the required python packages on your PC.

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

## SYSFW Board Config Generation {#BOARCFG_GEN}

\note Please refer this \ref RESOURCE_ALLOCATION_GUIDE guide if you want to modify the default resources.

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

- If you're not using any of the SBLs (SBL UART, SBL OSPI, SBL NULL) and is following the CCS boot method (\ref EVM_SOC_INIT_NOBOOT_MODE), make sure to build the sciclient_ccs_init application by doing

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
