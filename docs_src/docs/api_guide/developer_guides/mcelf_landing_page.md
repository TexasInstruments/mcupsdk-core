# Understanding Multicore ELF image format {#MCELF_LANDING}

[TOC]

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X

\note RPRC format has been deprecated from SDK 11.00 release onwards. MCELF is the default application format.

\endcond

\cond SOC_AM243X || SOC_AM64X

\note RPRC format has been deprecated from SDK 12.00 release onwards. MCELF is the default application format.

\endcond


## Introduction

Multicore ELF, or MCELF is a new application image format different from the older RPRC format.
Unlike a proprietary format like RPRC, multicore elf is based on the widely recognized ELF (Executable and Linkable format) standard.
This ensures seamless compatibility with industry-standard tools and platforms.

For more information on ELF format please refer https://man7.org/linux/man-pages/man5/elf.5.html

## RPRC vs MCELF

<table>
<tr>
    <th>Feature</th>
    <th>RPRC</th>
    <th>Multicore ELF</th>
</tr>
<tr>
    <td bgcolor=#F0F0F0>Generated using TI custom tools</td>
    <td>Yes
    <td>No proprietary tools in image generation flow
</tr>
<tr>
    <td bgcolor=#F0F0F0>Readable by standard ELF tools</td>
    <td>No
    <td>Yes
</tr>
<tr>
    <td bgcolor=#F0F0F0>Customizable metadata</td>
    <td>Not supported
    <td>Supported via PT_NOTE segment
</tr>
<tr>
    <td bgcolor=#F0F0F0>Customizable data segment sizes</td>
    <td>Not Supported
    <td>Supported
</tr>
<tr>
    <td bgcolor=#F0F0F0>XIP</td>
    <td>Supported
    <td>Supported
</tr>
<tr>
    <td bgcolor=#F0F0F0>Secure boot time</td>
    <td>Slow
    <td>Very fast
</tr>
</table>

## Generation of MCELF images

  \imageStyle{mcelf_intro.png,width:70%}
  \image html mcelf_intro.png "MCELF Image Generation"


Legend        | Description
------------- | -------------
C0/C1..       | Individual core ELFs
SSO           | Static shared object
ELF H         | ELF Header
S0/S2..       | Segment 1,2..
PHT           | Program Header Table

- When an application is built, a .out image is built for each core. These are ELF files themselves that contain data as segments within.
- The segments are extracted from the individual files and combined together into a single list.
- These segments represent chunks of data that need to be loaded into the memory when the program is executed.
- A segment contains the following information: 
  - Memory address: Address where the segment should be loaded
  - File offset: offset of the segment start from the ELF header
  - Memory size: Size of segment in memory after loading
  - Flags: If segment is of type Read(R)/Write(W)/Execute(X)
\cond SOC_AM263PX || SOC_AM261X || SOC_AM263X
- The sizes and order of these new segments can be controlled using suitable arguments to the genimage.py script.
\endcond
\cond SOC_AM243X || SOC_AM64X
- The sizes and order of these new segments can be controlled using suitable arguments to the genimage_am64x.py script.
\endcond
- The first and last segments are the note segments by default. It can be tailored according to the application via the tool.
- Once segment list is fixed, the Program header table is re-written based on the current segments and then the ELF header is regenerated based on updated PHT.

- For more information on the tool refer \ref MCELF_GEN_TOOL

## Booting MCELF images

- The SBL parses the image, loads the segments into memory and releases the core from reset.
Refer \ref BOOTFLOW_MCELF_BOOT.

- To boot mcelf images, SBLs are available with names as **sbl_xx_multicore_elf**.

- Please refer:

\cond SOC_AM263PX || SOC_AM261X || SOC_AM263X
  - \ref EXAMPLES_DRIVERS_SBL_CAN_MCELF
  - \ref EXAMPLES_DRIVERS_SBL_CAN_UNIFLASH_MCELF
\cond SOC_AM263X
  - \ref EXAMPLES_DRIVERS_SBL_QSPI_MCELF
\endcond
\cond SOC_AM263PX || SOC_AM261X
  - \ref EXAMPLES_DRIVERS_SBL_OSPI_MCELF
  - \ref EXAMPLES_DRIVERS_SBL_OSPI_FASTBOOT_MCELF
\endcond
  - \ref EXAMPLES_DRIVERS_SBL_UART_MCELF
\endcond

\cond SOC_AM64X || SOC_AM243X
  - \ref EXAMPLES_DRIVERS_SBL_SD
  - \ref EXAMPLES_DRIVERS_SBL_OSPI
  - \ref EXAMPLES_DRIVERS_SBL_UART
  - \ref EXAMPLES_DRIVERS_SBL_OSPI_MULTI_PARTITION
  - \ref EXAMPLES_DRIVERS_SBL_DFU_UNIFLASH
  - \ref EXAMPLES_DRIVERS_SBL_PCIE
\endcond
\cond SOC_AM64X
  - \ref EXAMPLES_DRIVERS_SBL_OSPI_LINUX
  - \ref EXAMPLES_DRIVERS_SBL_EMMC_LINUX
\endcond


## Limitations

- XLAT and SSO features are still under development.