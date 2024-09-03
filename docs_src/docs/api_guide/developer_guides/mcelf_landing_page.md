# Understanding Multicore ELF image format {#MCELF_LANDING}

[TOC]

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X

\note RPRC format would be deprecated from SDK 11.00 release onwards. MCELF would be the default application format going forward.

\endcond

## Introduction

Multicore ELF, or MCELF is a new application image format different from the existing RPRC format.
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
- The sizes and order of these new segments can be controlled using suitable arguments to the genimage.py script.
- The first and last segments are the note segments by default. It can be tailored according to the application via the tool.
- Once segment list is fixed, the Program header table is re-written based on the current segments and then the ELF header is regenerated based on updated PHT.

- For more information on the tool refer \ref MCELF_GEN_TOOL

## Booting MCELF images

- The steps for booting MCELF images are similar to RPRC. The SBL parses the image, loads the segments into memory and releases the core from reset.
Refer \ref BOOTFLOW_MCELF_BOOT.

- To boot mcelf images, special SBLs are available with names as **sbl_xx_multicore_elf**.

- Please refer:

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

\note 
- SBLs that supports parsing, loading and authenticating Multicore ELF (MCELF) are different from the one that supports RPRC format.
- SBLs and application formats are tightly coupled and cannot be used interchangeably. MCELF SBLs support booting only MCELF format images and not RPRC format images. Similarly, RPRC SBLs support booting only RPRC format images and not MCELF format images.
- SBLs with MCELF support have project names of the form **sbl_<bootmode>_multicore_elf** . Meanwhile, SBLs that support RPRC boot only have the boot mode specifier like **sbl_<bootmode>**.
  - For example,

    - sbl_uart is an SBL which only supports booting RPRC application image format.
    - sbl_uart_multicore_elf is an SBL which only supports booting MCELF application image format.


## Limitations

- TI Uniflash tool does not support flashing of .mcelf images currently.
- XLAT and SSO features are still under development.