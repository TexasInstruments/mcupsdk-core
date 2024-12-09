# Booting Tools {#TOOLS_BOOT}

[TOC]

\note To see the exact sequence of steps in which applications and secondary bootloader (SBL) are converted from compiler generated .out files to
      boot images, see the makefile `makefile_ccs_bootimage_gen` that is included in every example and secondary bootloader (SBL) CCS project.

\note If you are using makefile based build, then see the file named `makefile` in the example folder.

## Introduction

This section describes the various tools that are used to create boot images for all the SDK applications

## Important files and folders

<table>
<tr>
    <th>Folder/Files
    <th>Description
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/tools/boot/</td></tr>
<tr>
    <td>multicoreImageGen/
    <td>Tool to combine multiple RPRC into a single binary
</tr>
<tr>
    <td>out2rprc/
    <td>Tool to convert compiler generated ELF .out for a CPU to a compact and loadable binary representation, called RPRC.
</tr>
<tr>
    <td>sbl_prebuilt/
    <td>Pre-built secondary bootloader (SBL) images and flash configuration files for different supported EVMs, see also \ref TOOLS_FLASH
</tr>
<tr>
    <td>signing/
    <td>Security signing scripts need to create boot images that can be booted by ROM bootloader (RBL)
</tr>
\cond SOC_AM243X || SOC_AM64X
<tr>
    <td>xipGen/
    <td>Tool to split a RPRC file generated from `out2rprc` into two files containing non-XIP and XIP sections.
</tr>
\endcond
\if SOC_AM65X
<tr>
    <td>uart_bootloader.py
    <td>Python script used to send the SBL, system firmware(sysfw) and appimage binaries over UART using XMODEM protocol in UART boot mode
</tr>
<tr>
    <td>uart_uniflash.py
    <td>Python script used to flash SBL, system firmware(sysfw) and applications to EVM flash using UART. See \ref TOOLS_FLASH for more details.
</tr>
\else
<tr>
    <td>uart_bootloader.py
    <td>Python script used to send the SBL and appimage binaries over UART using XMODEM protocol in UART boot mode
</tr>
<tr>
    <td>uart_uniflash.py
    <td>Python script used to flash SBL and applications to EVM flash using UART. See \ref TOOLS_FLASH for more details.
</tr>
\endif
\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
<tr>
    <td>genimage.py
    <td>Python script used to generate multicore elf image from individual core elf images.
</tr>
\endcond
</table>

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X

\note RPRC tools (Out2RPRC and Multi-core Image Gen) would be deprecated from SDK 11.00 release onwards. MCELF would be the default application format going forward.

\endcond

## Out2RPRC {#OUT2RPRC_TOOL}

- This tool converts the application executable (.out) into custom TI RPRC (.rprc) image - an image loadable by the secondary bootloader (SBL).
- This tool strips out the initialized sections from the executable file (*.out) and places them in a compact format that the SBL can understand.
- The output RPRC file is typically much smaller than the original executable (*.out) file.
- The RPRC files are intermediate files in a format that is consumed by `MulticoreImageGen` tool that generates the final binary that is flashed (`*.appimage`)
- The RPRC file format contains header to various sections in the executable like section run address, size and
  a overall header which mentions the number of sections and the start offset to the first section.
- The RPRC magic word is `0x43525052` - which is ASCII equivalent for `RPRC`
- Shown below is the file header and section format for RPRC files.

    \imageStyle{tools_rprc_format.png,width:40%}
    \image html tools_rprc_format.png "RPRC File Format"

- Given below are the structs used in the bootloader library to parse an RPRC image

```C
typedef struct Bootloader_RprcFileHeader_s
{
    uint32_t magic;
    uint32_t entry;
    uint32_t rsvdAddr;
    uint32_t sectionCount;
    uint32_t version;

} Bootloader_RprcFileHeader;

typedef struct Bootloader_RprcSectionHeader_s
{
    uint32_t addr;
    uint32_t rsvdAddr;
    uint32_t size;
    uint32_t rsvdCrc;
    uint32_t rsvd;

} Bootloader_RprcSectionHeader;
```

- This tool is provided as a minified JS script. To convert the application executable into RPRC image file, it can be used as
  \code
  cd ${SDK_INSTALL_PATH}/tools/boot/out2rprc
  ${NODE} elf2rprc.js {input application executable file (.out)}
  \endcode

- RPRC mandates that the sections in the application image should be 8-byte aligned. Make sure that this is taken care
  in the linker.cmd file. Sample:
```
GROUP {
        .text.hwi: palign(8)
        .text.cache: palign(8)
        .text.mpu: palign(8)
        .text.boot: palign(8)
        .text:abort: palign(8) /* this helps in loading symbols when using XIP mode */
    } > OCRAM
```

## Multi-core Image Gen {#MULTICOREIMAGEGEN_TOOL}

- This tool converts the RPRC files created for each CPU into a single combined multicore application image that can be booted by the secondary bootloader (SBL)
- Shown below is the file format for multicore image files.

  \imageStyle{tools_multicore_format.png,width:60%}
  \image html tools_multicore_format.png "Multi-core Image File Format"

- Given below are the structs used in the bootloader library for parsing multicore images:

```C
typedef struct Bootloader_MetaHeaderStart_s
{
    uint32_t magicStr;
    uint32_t numFiles;
    uint32_t devId;
    uint32_t rsvd;

} Bootloader_MetaHeaderStart;

typedef struct Bootloader_MetaHeaderCore_s
{
    uint32_t coreId;
    uint32_t imageOffset;

} Bootloader_MetaHeaderCore;

typedef struct Bootloader_MetaHeaderEnd_s
{
    uint32_t rsvd;
    uint32_t magicStringEnd;

} Bootloader_MetaHeaderEnd;
```
- The number of core meta headers present is equal to the number of cores included.
- The meta header magic word is `0x5254534D` - which is ASCII equivalent for `MSTR`
- In Windows or Linux, use the following command to convert RPRC images into a multicore `.appimage` file
    \code
    cd ${SDK_INSTALL_PATH}/tools/boot/multicoreImageGen
    ${NODE} multicoreImageGen.js --devID {DEV_ID} --out {Output image file (.appimage)} {core 1 rprc file}@{core 1 id} [ {core n rprc file}@{core n id} ... ]
    \endcode

\cond SOC_AM64X || SOC_AM243X

- In case of @VAR_SOC_NAME, `DEV_ID` is `55`.
- The various core ID to be used are as below.

CORE        | CORE ID
------------|--------
r5fss0-0    | 4
r5fss0-1    | 5
r5fss1-0    | 6
r5fss1-1    | 7
m4fss0-0    | 14

\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X

- In case of @VAR_SOC_NAME, `DEV_ID` is `55`.
- The various core ID to be used are as below.

CORE        | CORE ID
------------|--------
r5fss0-0    | 0
r5fss0-1    | 1
r5fss1-0    | 2
r5fss1-1    | 3

\endcond

\cond SOC_AM273X

- In case of @VAR_SOC_NAME, `DEV_ID` is `55`.
- The various core ID to be used are as below.

CORE        | CORE ID
------------|--------
r5fss0-0    | 0
r5fss0-1    | 1
c66xdsp_0   | 2

\endcond

\cond SOC_AWR294X

- In case of @VAR_SOC_NAME, `DEV_ID` is `55`.
- The various core ID to be used are as below.

CORE        | CORE ID
------------|--------
r5fss0-0    | 0
r5fss0-1    | 1
c66xdsp_0   | 2
r4          | 3

\endcond

\cond SOC_AM65X

- In case of @VAR_SOC_NAME, `DEV_ID` is `55`.
- The various core ID to be used are as below.

CORE        | CORE ID
------------|--------
r5fss0-0    | 4
r5fss0-1    | 5

\endcond

## MCELF Image Gen {#MCELF_GEN_TOOL}

- This tool takes individual core ELF files as input and combines their segments to create a single ELF file.

- Shown below is the file format for an mcelf image file and its metacontent as seen by `readelf`

  \imageStyle{tools_mcelf_format.png,width:60%}
  \image html tools_mcelf_format.png "MCELF Image File Format"

- Segment data from each input ELF file is extracted and appended together to form a single list of segments.
- The program header table fields are then re-calculated and the header table is regenerated using information from this new segment list.
- Once that is done, from the updated program header, the ELF header is regenerated.
- The first segment is the note segment. It can be customized to store information according to your application. By default the note segment contains vendor information, segment to core mapping and entry points.
- The segment sizes can be manipulated using appropriate arguments.


  Argument                | Description
  ------------------------|-------------------
  `--core-img`            | Path to individual binaries of each core. It is a mandatory argument. Input is given in this format: `--core-img=0:<core0_binary.out> --core-img=1:<core1_binary.out>`
  `--output`              | The output file name. It is a mandatory argument. `--output=<file_name>.mcelf`
  `--merge-segments`      | Enable merging segments based on a tolerance limit. Default value is false.
  `--tolerance-limit`     | The maximum difference (in bytes) between the end address of previous segment and start address of current segment for merging the segments. Default value is zero.
  `--ignore-context`      | Enable merging of segments that are of different cores. Default value is false.
  `--xip`                 | XIP section's start and end address seperated by a colon. It creates a new file `<filename>.mcelf_xip`. Default value is 'none' (XIP is disabled). To enable XIP creation: `--xip=0x60100000:0x60200000`
  `--max_segment_size`    | Maximum allowed size of a loadable segment. This feature can only be used with merge_segments disabled. Default value is 8192 bytes.
  `--xlat`                | SOC specific Address Translation. (Under development, reserved for future use)
  `--sso`                 | Shared static objects. (Under development, reserved for future use)

- The input for arguments 3-7 are defined in {MCU_SDK_PATH}/devconfig/devconfig.mak file.

- Given below are the structs used in the bootloader library to parse a 32 bit MCELF binary

```C
#define E_IDENT 16

/* ELF HEADER */
typedef struct Bootloader_ELFH32_s
{
    uint8_t  e_ident[E_IDENT];
    uint16_t e_type;
    uint16_t e_machine;
    uint32_t e_version;
    uint32_t e_entry;
    uint32_t e_phoff;
    uint32_t e_shoff;
    uint32_t e_flags;
    uint16_t e_ehsize;
    uint16_t e_phentsize;
    uint16_t e_phnum;
    uint16_t e_shentsize;
    uint16_t e_shnum;
    uint16_t e_shstrndx;

} Bootloader_ELFH32;

/* PROGRAM HEADER */
typedef struct Bootloader_ELFPH32_s
{
	uint32_t type;
    uint32_t offset;
    uint32_t vaddr;
    uint32_t paddr;
    uint32_t filesz;
    uint32_t memsz;
    uint32_t flags;
    uint32_t align;

} Bootloader_ELFPH32;

/* NOTE SEGMENT */
typedef struct Bootloader_ELFNote_s
{
    uint32_t namesz;
    uint32_t descsz;
    uint32_t type;
} Bootloader_ELFNote;
```

- Use the following command to invoke this script to generate a basic `.mcelf` image from input `.out` files without any segment manipulations

```bash
$ cd tools/boot/multicore-elf

$ pip install -r requirements.txt

$ {PYTHON} genimage.py --core-img={CORE_0_ID}:{core0_app.out} --core-img={CORE_1_ID}:{core1_app.out} --core-img={CORE_2_ID}:{core2_app.out} --core-img={CORE_3_ID}:{core3_app.out} --output={application.mcelf} 
```

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X

- The various core ID to be used are as below.

CORE        | CORE ID
------------|--------
r5fss0-0    | 0
r5fss0-1    | 1
r5fss1-0    | 2
r5fss1-1    | 3

\endcond

\cond SOC_AM64X || SOC_AM243X || SOC_AM65X
## XIP Image Generator Tool

- This tool, splits a input RPRC application file, into two RPRC files,
   - First RPRC file, containing non-XIP sections and the application entry point
   - Second RPRC file, containing XIP sections. Here, adjacent sections are merged into one section

- The final number of sections in both RPRC files taken together can be less
  than the sections in the input file due to section merging in the XIP RPRC file.

  \imageStyle{tools_xip_gen.png,width:60%}
  \image html tools_xip_gen.png "XIP Image Generation Tool flow"


- The non-XIP RPRC file should be flashed and booted via SBL as usual

- The XIP RPRC file should be flashed via the SDK flash writer, using the command `--flash-xip`,
  the flash writer in this case will flash sections at the flash address mentioned in the RPRC section header.

- To see the detailed options supported by the tool, run this tool with the option `--help`.
  Example, output in Windows is shown below,

  \code
  > cd {SDK_INSTALL_PATH}/tools/boot/xipGen
  > xipGen.exe

    XIP Image Creation Tool  - (c) Texas Instruments 2021, created on Apr 19 2021

    Usage: xipGen [options]

    Description,
    This tool, splits a input RPRC application file, into two RPRC files,
    - First RPRC file, containing non-XIP sections. The application entry point is assumed
        to be in non-XIP region.
    - Second RPRC file, containing XIP sections. The RPRC entry point is set to 0 and
        adjacent XIP sections are merged

    Note, the final number of sections in both RPRC files taken together can be less
    than the sections in the input file due to section merging in the XIP RPRC file.

    The non-XIP RPRC file should be flashed and booted via SBL as usual

    The XIP RPRC file should be flashed via the SDK flash writer, the flash writer will
    flash sections at the flash address mentioned in the RPRC sections

    Options,
    --input, -i : input RPRC file,
    --output, -o : output RPRC file of non-XIP sections,
    --output-xip, -x : output RPRC file of XIP sections,
    --flash-start-addr, -f : XIP flash address space start, specified in hex. If not specified 0x60000000 is used
    --flash-size, -s : XIP flash address space size in units of mega bytes, specified as integer. If not specified 256 MB is used
    --verbose, -v : Verbose prints are enabled during the tool execution
    --help, -h : Shows this help
  \endcode
\endcond


## UART Bootloader Python Script {#UART_BOOTLOADER_PYTHON_SCRIPT}

\if SOC_AM65X
- This script is used in UART boot mode for sending the SBL, system firmware(sysfw) and appimage binaries to the EVM via UART using XMODEM protocol
\else
- This script is used in UART boot mode for sending the SBL and appimage binaries to the EVM via UART using XMODEM protocol
\endif
- Make sure that python3 and its dependent modules are installed in the host machine as mentioned in \ref INSTALL_PYTHON3
- Booting via UART is slow, but is useful if application loading via CCS or OSPI boot is not an option
- Make sure the UART port used for terminal is identified as mentioned in \ref CCS_UART_TERMINAL
- Make sure you have the EVM power cable and UART cable connected as shown in \ref EVM_CABLES
- To boot applications using this script, **POWER OFF the EVM**
- Switch to \ref BOOTMODE_UART.
- **POWER ON the EVM**
- To confirm that the board is in UART boot mode, open the UART terminal and confirm that you see the character 'C' getting printed on the console every 2-3 seconds.
- Now close the terminal. This is important as the script won't be able to function properly if the UART terminal is open.
- Open a command prompt and run the below command to send the SBL and application binary to the EVM
  \if SOC_AM65X
  \code
  cd ${SDK_INSTALL_PATH}/tools/boot
  python uart_bootloader.py -p COM<x> --bootloader=sbl_prebuilt/{board}/default_sbl_uart.cfg
  \endcode
  \else
    \code
  cd ${SDK_INSTALL_PATH}/tools/boot
  python uart_bootloader.py -p COM<x> --bootloader=sbl_prebuilt/{board}/sbl_uart.release.tiimage --file=< path to multicore appimage of application binary >
  \endcode
  \endif
\if SOC_AM65X
- When you execute this, the script first sends the uart bootloader, system firmware(sysfw) and then the multicore appimage
\else
- When you execute this, the script first sends the uart bootloader, and then the multicore appimage
\endif
- After the multicore appimage is successfully parsed, the uart bootloader sends an acknowledgment to the script
and waits for 5 seconds before running the application binary
- Upon receiving the ack, the script will exit successfully
- Connect to the UART terminal within 5 seconds to see logs from the application
- Below are the logs of the script after all the files have been sent
  \if SOC_AM65X
  \code
  Parsing config file ...
  Parsing config file ... SUCCESS. Found 3 command(s) !!!

  Executing command 1 of 3 ...
  Found the UART bootloader ... sending sbl_prebuilt/am65x-idk/sbl_uart.release.tiimage
  Sent bootloader sbl_prebuilt/am65x-idk/sbl_uart.release.tiimage of size 48016 bytes in 6.47s.

  Executing command 2 of 3 ...
  Command arguments : --file=../../source/drivers/sciclient/soc/am65x/sysfw_sr2.bin
  Sent ../../source/drivers/sciclient/soc/am65x/sysfw_sr2.bin of size 263083 bytes in 28.8s.
  [STATUS] Application load SUCCESS !!!

  Executing command 3 of 3 ...
  Command arguments : --file=../../examples/drivers/i2c/i2c_led_blink/am65x-idk/r5fss0-0_freertos/ti-arm-clang/i2c_led_blink.release.appimage
  Sending ../../examples/drivers/i2c/i2c_led_blink/am65x-idk/r5fss0-0_freertos/ti-arm-clang/i2c_led_blink.release.appimage: 56595bytes
  Sent ../../examples/drivers/i2c/i2c_led_blink/am65x-idk/r5fss0-0_freertos/ti-arm-clang/i2c_led_blink.release.appimage of size 56012 bytes in 8.44s.
  [STATUS] Application load SUCCESS !!!

  Sent End Of File Transfer message of size 4 bytes in 2.94s.

  All commands from config file are executed !!!
  Connect to UART in 5 seconds to see logs from UART !!!
  \endcode
  \else
  \code
  Sending the UART bootloader sbl_prebuilt/{board}/sbl_uart.release.tiimage ...
  Sent bootloader sbl_prebuilt/{board}/sbl_uart.release.tiimage of size 243975 bytes in 23.94s.

  Sending the application ../../examples/drivers/udma/udma_memcpy_polling/{board}/r5fss0-0_nortos/ti-arm-clang/udma_memcpy_polling.release.appimage ...
  Sent application ../../examples/drivers/udma/udma_memcpy_polling/{board}/r5fss0-0_nortos/ti-arm-clang/udma_memcpy_polling.release.appimage of size 99580 bytes in 11.74s.
  [STATUS] Application load SUCCESS !!!
  Connect to UART in 2 seconds to see logs from UART !!!
  \endcode
  \endif


\cond SOC_AM243X || SOC_AM64X || SOC_AM261X

## USB Bootloader Python Script {#USB_BOOTLOADER}

- This script is used in DFU boot mode for sending the SBL and appimage binaries to the EVM via USB DFU.
- Make sure that \ref INSTALL_DFU_UTIL tool is installed properly and the DFU enumeration is verified.
\cond SOC_AM261X 
- The source code for DFU Utils tool is available in GitHub (https://github.com/TexasInstruments/dfu-util) and Prebuilt binary is packaged in SDK.
\endcond
- Make sure that python3 and its dependent modules are installed in the host machine as mentioned in \ref INSTALL_PYTHON3
- Change the boot mode to DFU boot mode \ref EVM_SETUP_PAGE
- **POWER cycle the EVM**
- Open a command prompt and run the below command to send the SBL and application binary to the EVM
\cond SOC_AM243X || SOC_AM64X 
  \code
  cd ${SDK_INSTALL_PATH}/tools/boot
  python usb_bootloader.py --bootloader=sbl_prebuilt/{board}/sbl_dfu.release.hs_fs.tiimage --file=< path to multicore appimage of application binary
  \endcode
\endcond
\cond SOC_AM261X 
  \code
  cd ${SDK_INSTALL_PATH}/tools/boot
  python usb_bootloader.py --bootloader=sbl_prebuilt/{board}/sbl_dfu.release.hs_fs.tiimage --file=< path to multicore appimage of application binary --use-sdk-utility
  \endcode
- This **--use-sdk-utility** option selects the custom DFU util from the SDK to perform DFU operations.
\endcond
- When you execute this, the script first sends the SBL USB bootloader, and then the multicore appimage
- Connect to the UART terminal to see the booting information
- Below are the logs of the script after all the files have been sent

\code
	INFO: Bootloader_loadSelfCpu:207: CPU r5f0-0 is initialized to 800000000 Hz !!!
	INFO: Bootloader_loadSelfCpu:207: CPU r5f0-1 is initialized to 800000000 Hz !!!
	[BOOTLOADER_PROFILE] Boot Media       : USB DFU
	[BOOTLOADER_PROFILE] Boot Image Size  : 114 KB
	[BOOTLOADER_PROFILE] Cores present    :
	m4f0-0
	r5f1-0
	r5f1-1
	r5f0-0
	r5f0-1
	[BOOTLOADER PROFILE] CPU load                         :     200191us
	[BOOTLOADER_PROFILE] SBL Total Time Taken             :     200192us

	Image loading done, switching to application ...
	INFO: Bootloader_runCpu:155: CPU m4f0-0 is initialized to 400000000 Hz !!!
	INFO: Bootloader_runCpu:155: CPU r5f1-0  is initialized to 800000000 Hz !!!
	INFO: Bootloader_runCpu:155: CPU r5f1-1 is initialized to 800000000 Hz !!!
	INFO: Bootloader_runSelfCpu:217: All done, reseting self ...

	[IPC NOTIFY ECHO] Message exchange started by main core !!!
	[m4f0-0]     0.030020s : [IPC NOTIFY ECHO] Remote Core waiting for messages from main core ... !!!
	[r5f0-1]     0.002099s : [IPC NOTIFY ECHO] Remote Core waiting for messages from main core ... !!!
	[r5f0-1]     2.338054s : [IPC NOTIFY ECHO] Remote core has echoed all messages !!!
	[r5f1-0]     0.022147s : [IPC NOTIFY ECHO] Remote Core waiting for messages from main core ... !!!
	[r5f1-0]     2.358900s : [IPC NOTIFY ECHO] Remote core has echoed all messages !!!
	[r5f1-1]     0.015147s : [IPC NOTIFY ECHO] Remote Core waiting for messages from main core ... !!!
	[r5f1-1]     2.351658s : [IPC NOTIFY ECHO] Remote core has echoed all messages !!!
	[IPC NOTIFY ECHO] All echoed messages received by main core from 4 remote cores !!!
	[IPC NOTIFY ECHO] Messages sent to each core = 1000000
	[IPC NOTIFY ECHO] Number of remote cores = 4
	All tests have passed!!
	[m4f0-0]     3.568946s : [IPC NOTIFY ECHO] Remote core has echoed all messages !!!
\endcode

- Refer \ref EXAMPLES_DRIVERS_SBL_DFU

\endcond


\cond SOC_AM64X || SOC_AM65X

## Linux Appimage Generator Tool {#LINUX_APPIMAGE_GEN_TOOL}

- This tool generates a Linux Appimage by taking the Linux binaries (ATF, OPTEE, SPL) as input and generates a Linux appimage containing the input Linux binaries.
- The input file location can be mentioned in the `config.mak` file located at {SDK_INSTALL_PATH}/tools/boot/linuxAppimageGen
    - `PSDK_LINUX_PREBUILT_IMAGES=$(PSDK_LINUX_PATH)/board-support/prebuilt-images`
- The input file names for ATF, OPTEE and SPL can also be mentioned in the `config.mak` file.
    - `#Input linux binaries`\n
       `ATF_BIN_NAME=bl31.bin`\n
       `OPTEE_BIN_NAME=bl32.bin`\n
       \if SOC_AM65X
       `SPL_BIN_NAME=u-boot-spl.bin-am65xx-evm`\n
       \else
       `SPL_BIN_NAME=u-boot-spl.bin-am64xx-evm`\n
       \endif
- The load address for ATF, OPTEE and SPL need to be mentioned in the `config.mak` file.
    - `#Linux image load address`\n
      `ATF_LOAD_ADDR=0x0701a0000`\n
      `OPTEE_LOAD_ADDR=0x9e800000`\n
      `SPL_LOAD_ADDR=0x80080000`\n
- The output appimage name can be mentioned in the `config.mak` file.
    - `#Output appimage name`\n
      `LINUX_BOOTIMAGE_NAME=linux.appimage`\n
- Run the makefile at {SDK_INSTALL_PATH}/tools/boot/linuxAppimageGen to generate the Linux appimage
    - For Windows
        \code
        cd ${SDK_INSTALL_PATH}/tools/boot/linuxAppimageGen
        gmake -s all
        \endcode
    - For Linux
        \code
        cd ${SDK_INSTALL_PATH}/tools/boot/linuxAppimageGen
        make -s all
        \endcode
- The Linux appimage wil be generated at {SDK_INSTALL_PATH}/tools/boot/linuxAppimageGen after running the makefile
\endcond

\cond SOC_AM64X || SOC_AM243X || SOC_AM263X || SOC_AM263PX || SOC_AM261X

## SoC ID parser Python Script {#SOC_ID_PARSER}

- Boot ROM reports SoC ID on UART console of the device when UART boot mode is selected. It reports on both GP and HS devices and it provides insights into device configuration which would be helpful for debugs.

- uart_boot_socid.py is a python3 based parser to convert the hexadecimal numbers reported by ROM to human readable text, below are the steps involved to use this parser. This will be helpful in debugging the device boot issue. This will also help to see important information about device like device type, prime/non-prime, key count, key revision, MPK hash etc.

- Make sure python3 is installed as mentioned in \ref INSTALL_PYTHON3

- **Steps to use the parser:**

    - Copy the soc id reported in UART console

    - pass the copied id to the script either as a plain string or saving it into a file

    - execute the script
    \code
    $python uart_boot_socid.py -d am64x --string=<copied soc id>
    or
    $python uart_boot_socid.py -d am64x --file=soc_id.txt
    \endcode


- Example Output:
    \code
    -----------------------
    SoC ID Header Info:
    -----------------------
    NumBlocks            : 2
    -----------------------
    SoC ID Public ROM Info:
    -----------------------
    SubBlockId           : 1
    SubBlockSize         : 26
    DeviceName           : am64x
    DeviceType           : HSSE
    DMSC ROM Version     : [0, 2, 0, 0]
    R5 ROM Version       : [0, 2, 0, 0]
    -----------------------
    SoC ID Secure ROM Info:
    -----------------------
    Sec SubBlockId       : 2
    Sec SubBlockSize     : 166
    Sec Prime            : 0
    Sec Key Revision     : 1
    Sec Key Count        : 1
    Sec TI MPK Hash      : b018658ad99dc903c8c9bfb27b12751099920a042ad1dfea7b7ba57369f15546de285edde6a7b39a8bdc40a27b237f8fb1e57f245e80b929c1e28b024aa2ecc6
    Sec Cust MPK Hash    : 1f6002b07cd9b0b7c47d9ca8d1aae57b8e8784a12f636b2b760d7d98a18f189760dfd0f23e2b0cb10ec7edc7c6edac3d9bdfefe0eddc3fff7fe9ad875195527d
    Sec Unique ID        : 01f22176afca3a82692ce53b2738b8c982f7538602871e0bdb7dc2f7668d04b2
    \endcode

\cond SOC_AM243X

\note The DeviceName will show as am64x in the am243x devices because the Public ROM is same for both am243x and am64x devices.

\endcond

\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
## CAN Bootloader Python Script {#CAN_BOOTLOADER_PYTHON_SCRIPT}

- This script is used in QSPI boot mode for sending the appimage binaries to the EVM via CAN, after flashing the SBL CAN. Refer \ref BASIC_STEPS_TO_FLASH_FILES for flashing.
- Make sure that python3 and its dependent modules are installed in the host machine as mentioned in \ref INSTALL_PYTHON3
- Make sure the UART port used for terminal is identified as mentioned in \ref CCS_UART_TERMINAL
- Make sure you have the EVM power cable and CAN cable connected as shown in \ref EXAMPLES_DRIVERS_SBL_CAN
- To boot applications using this script, **POWER OFF the EVM**
\cond SOC_AM263X 
- Switch to \ref BOOTMODE_QSPI
\endcond
\cond SOC_AM263PX || SOC_AM261X
- Switch to \ref BOOTMODE_OSPI
\endcond
- **POWER ON the EVM**
- Open a command prompt and run the below command to send the application binary to the EVM
\code
cd ${SDK_INSTALL_PATH}/tools/boot
python can_bootloader.py --file=< path to multicore appimage of application binary >
\endcode
- When you execute this, the script first sends the multicore appimage to the EVM
- After the multicore appimage is successfully parsed, the CAN bootloader sends an acknowledgment to the script
- Upon receiving the ack, the script will exit successfully
- Connect to the UART terminal to see logs from the application
- Below are the logs of the script after all the files have been sent
  \code
  Sending the application ../../examples/drivers/udma/udma_memcpy_polling/{board}/r5fss0-0_nortos/ti-arm-clang/udma_memcpy_polling.release.appimage ...
  Sent application ../../examples/drivers/udma/udma_memcpy_polling/{board}/r5fss0-0_nortos/ti-arm-clang/udma_memcpy_polling.release.appimage of size 99580 bytes in 11.74s.
  [STATUS] BOOTLOADER_CAN_STATUS_LOAD_SUCCESS!!!
  Connect to UART to see logs from UART !!!
  \endcode
\endcond
