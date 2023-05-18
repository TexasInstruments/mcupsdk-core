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
<tr>
    <td>uart_bootloader.py
    <td>Python script used to send the SBL and appimage binaries over UART using XMODEM protocol in UART boot mode
</tr>
<tr>
    <td>uart_uniflash.py
    <td>Python script used to flash SBL and applications to EVM flash using UART. See \ref TOOLS_FLASH for more details.
</tr>
</table>

## Out2RPRC

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

## Multi-core Image Gen

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

\cond SOC_AM263X

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


## Signing Scripts {#TOOLS_BOOT_SIGNING}

- To run these scripts, one needs `openssl` installed as mentioned here, \ref INSTALL_OPENSSL
- Signing scripts are a collection of scripts needed to sign ROM images (image booted by ROM - mostly the SBL) and application images (image booted by the SBL)
- The RBL requires the boot image (mostly SBL), to be signed always, even if we are not using secure boot.
- Use the following command to sign the SBL image.
    \code
    cd ${SDK_INSTALL_PATH}/tools/boot/signing
    x509CertificateGen.ps1 -b {BOOTIMAGE_BIN_NAME} -o {BOOTIMAGE_NAME} -c R5 -l {SBL_RUN_ADDRESS} -k {BOOTIMAGE_CERT_KEY} -d DEBUG -j DBG_FULL_ENABLE -m SPLIT_MODE
    \endcode
- In Windows, use powershell to execute the script file.
    \code
    powershell -executionpolicy unrestricted -command x509CertificateGen.ps1
    \endcode
\cond SOC_AM64X || SOC_AM243X
- We follow a combined boot method for ROM images. Here the ROM Bootloader (RBL) boots the SBL, SYSFW and BOARDCFG together. The boot image would be a binary concatenation of x509 Certificate, SBL, SYSFW, BOARDCFG (and the SYSFW inner certificate in case of HS device) binary blobs. We use a python script to generate this final boot image. This script has a dependency on `openssl` as mentioned before, so make sure you've installed it. To generate a combined boot image, one can do as below:

- For GP devices
  \code
  cd ${SDK_INSTALL_PATH}/tools/boot/signing
  ${PYTHON} rom_image_gen.py --swrv 1 --sbl-bin <path-to-sbl-binary> --sysfw-bin <path-to-sysfw-binary> --boardcfg-blob <path-to-boardcfg-binary-blob> --sbl-loadaddr ${SBL_RUN_ADDRESS} --sysfw-loadaddr ${SYSFW_LOAD_ADDR} --bcfg-loadaddr ${BOARDCFG_LOAD_ADDR} --key ${BOOTIMAGE_CERT_KEY} --rom-image <path-to-output-image>
  \endcode

- For HS devices, we have to pass the HS SYSFW binaries and also the SYSFW inner certificate to the signing script.
  \code
  cd ${SDK_INSTALL_PATH}/tools/boot/signing
  ${PYTHON} rom_image_gen.py --swrv 1 --sbl-bin <path-to-sbl-binary> --sysfw-bin <path-to-sysfw-binary> --sysfw-inner-cert <path-to-sysfw-inner-cert-binary> --boardcfg-blob <path-to-boardcfg-binary-blob> --sbl-loadaddr ${SBL_RUN_ADDRESS} --sysfw-loadaddr ${SYSFW_LOAD_ADDR} --bcfg-loadaddr ${BOARDCFG_LOAD_ADDR} --key ${BOOTIMAGE_CERT_KEY} --debug DBG_FULL_ENABLE --rom-image <path-to-output-image>
  \endcode

- By default SBLs provided in SDK are signed with full debug enable since this is needed for development. You can see from `--debug` switch used above. Once moved to production please remove this switch from the makefile.

- For SBL images or examples which is loaded by SBL, we use a different signing script. This is solely because of the x509 certificate template differences between ROM and SYSFW. In GP devices appimages are not signed. The signing happens only in HS devices. The script usage is:
  \code
  cd ${SDK_INSTALL_PATH}/tools/boot/signing
  $(PYTHON) appimage_x509_cert_gen.py --bin <path-to-the-binary> --authtype 1 --key <signing-key-derived-from-devconfig> --output <output-image-name>
  \endcode

- In the case of encryption, two extra options are also passed to the script like so:
  \code
  cd ${SDK_INSTALL_PATH}/tools/boot/signing
  $(PYTHON) appimage_x509_cert_gen.py --bin <path-to-the-binary> --authtype 1 --key <signing-key-derived-from-devconfig> --enc y --enckey <encryption-key-derived-from-devconfig> --output <output-image-name>
  \endcode

- These scripts are invoked in makefiles, and the image generation happens automatically along with the example build. So mostly these scripts need not be manually run.
\endcond


 - Here,
\cond SOC_AM64X || SOC_AM243X
  - `SBL_RUN_ADDRESS` is `0x70000000`
  - In the case of GP device, `BOOTIMAGE_CERT_KEY` is `rom_degenerateKey.pem`
  - In the case of HS device, `BOOTIMAGE_CERT_KEY` is `custMpk_am64x_am243x.pem`. For more details about this see \ref SECURE_BOOT
\endcond
\cond SOC_AM263X
  - `SBL_RUN_ADDRESS` is `0x70002000`
  - In the case of GP device, `BOOTIMAGE_CERT_KEY` is `am263x_gpkey.pem`
\endcond

\cond SOC_AM64X || SOC_AM243X
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

- This script is used in UART boot mode for sending the SBL and appimage binaries to the EVM via UART using XMODEM protocol
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
  \code
  cd ${SDK_INSTALL_PATH}/tools/boot
  python uart_bootloader.py -p COM<x> --bootloader=sbl_prebuilt/{board}/sbl_uart.release.tiimage --file=< path to multicore appimage of application binary >
  \endcode
- When you execute this, the script first sends the uart bootloader, and then the multicore appimage
- After the multicore appimage is successfully parsed, the uart bootloader sends an acknowledgment to the script
and waits for 5 seconds before running the application binary
- Upon receiving the ack, the script will exit successfully
- Connect to the UART terminal within 5 seconds to see logs from the application
- Below are the logs of the script after all the files have been sent
  \code
  Sending the UART bootloader sbl_prebuilt/{board}/sbl_uart.release.tiimage ...
  Sent bootloader sbl_prebuilt/{board}/sbl_uart.release.tiimage of size 243975 bytes in 23.94s.

  Sending the application ../../examples/drivers/udma/udma_memcpy_polling/{board}/r5fss0-0_nortos/ti-arm-clang/udma_memcpy_polling.release.appimage ...
  Sent application ../../examples/drivers/udma/udma_memcpy_polling/{board}/r5fss0-0_nortos/ti-arm-clang/udma_memcpy_polling.release.appimage of size 99580 bytes in 11.74s.
  [STATUS] Application load SUCCESS !!!
  Connect to UART in 2 seconds to see logs from UART !!!
  \endcode


\cond SOC_AM243X || SOC_AM64X

## USB Bootloader Python Script {#USB_BOOTLOADER}

- This script is used in DFU boot mode for sending the SBL and appimage binaries to the EVM via USB DFU.
- Make sure that \ref INSTALL_DFU_UTIL tool is installed properly and the DFU enumeration is verified.
- Make sure that python3 and its dependent modules are installed in the host machine as mentioned in \ref INSTALL_PYTHON3
- Change the boot mode to DFU boot mode \ref EVM_SETUP_PAGE
- **POWER cycle the EVM**
- Open a command prompt and run the below command to send the SBL and application binary to the EVM
  \code
  cd ${SDK_INSTALL_PATH}/tools/boot
  python usb_bootloader.py --bootloader=sbl_prebuilt/{board}/sbl_dfu.release.hs_fs.tiimage --file=< path to multicore appimage of application binary
  \endcode
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


\cond SOC_AM64X

## Linux Appimage Generator Tool {#LINUX_APPIMAGE_GEN_TOOL}

- This tool generates a Linux Appimage by taking the Linux binaries (ATF, OPTEE, SPL) as input and generates a Linux appimage containing the input Linux binaries.
- The input file location can be mentioned in the `config.mak` file located at {SDK_INSTALL_PATH}/tools/boot/linuxAppimageGen
    - `PSDK_LINUX_PREBUILT_IMAGES=$(PSDK_LINUX_PATH)/board-support/prebuilt-images`
- The input file names for ATF, OPTEE and SPL can also be mentioned in the `config.mak` file.
    - `#Input linux binaries`\n
       `ATF_BIN_NAME=bl31.bin`\n
       `OPTEE_BIN_NAME=bl32.bin`\n
       `SPL_BIN_NAME=u-boot-spl.bin-am64xx-evm`\n
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

\cond SOC_AM64X || SOC_AM243X

## SoC ID parser Python Script {#SOC_ID_PARSER}

- Boot ROM reports SoC ID on UART console of the device when UART boot mode is selected. It reports on both GP and HS devices and it provides insights into device configuration which would be helpful for debugs.

- uart_boot_socid.py is a python based parser to convert the hexadecimal numbers reported by ROM to human readable text, below are the steps involved to use this parser. This will be helpful in debugging the device boot issue. This will also help to see important information about device like device type, prime/non-prime, key count, key revision, MPK hash etc.

- Steps to use the parser:

1) Copy the soc id reported in UART console to socid.txt (socid.txt)

2) execute

\code
$python uart_boot_socid.py socid.txt
\endcode

- Example Output:

SoC ID Header Info: \n  \n

NumBlocks : [2] \n   \n

SoC ID Public ROM Info: \n \n

SubBlockId : 1 \n
SubBlockSize : 26 \n
DeviceName : am64x  \n
DeviceType : HSSE \n
DMSC ROM Version : [0, 1, 1, 1] \n
R5 ROM Version : [0, 1, 1, 1] \n \n

SoC ID Secure ROM Info: \n \n

Sec SubBlockId : 2 \n
Sec SubBlockSize : 166 \n
Sec Prime : 0 \n
Sec Key Revision : 1 \n
Sec Key Count : 1 \n
Sec TI MPK Hash : aa1f8e3095042e5c71ac40ede5b4e8c85fa87e03305ae0ea4f47933e89f4164aeb5a12ae13778f49de0622c1a578e6e747981d8c44a130f89a336a887a7955ee \n
Sec Cust MPK Hash : 1f6002b07cd9b0b7c47d9ca8d1aae57b8e8784a12f636b2b760d7d98a18f189760dfd0f23e2b0cb10ec7edc7c6edac3d9bdfefe0eddc3fff7fe9ad875195527d \n
Sec Unique ID : fd6e232b89dfc6ea125c2fa09f25f95034e08a54797490c32bf47c64bf4c8f21 \n
\cond SOC_AM243X

\note The DeviceName will show as am64x in the am243x devices because the Public ROM is same for both am243x and am64x devices.

\endcond

\endcond

\cond SOC_AM263X
## CAN Bootloader Python Script {#CAN_BOOTLOADER_PYTHON_SCRIPT}

- This script is used in QSPI boot mode for sending the appimage binaries to the EVM via CAN, after flashing the SBL CAN. Refer \ref BASIC_STEPS_TO_FLASH_FILES for flashing.
- Make sure that python3 and its dependent modules are installed in the host machine as mentioned in \ref INSTALL_PYTHON3
- Make sure the UART port used for terminal is identified as mentioned in \ref CCS_UART_TERMINAL
- Make sure you have the EVM power cable and CAN cable connected as shown in \ref EXAMPLES_DRIVERS_SBL_CAN
- To boot applications using this script, **POWER OFF the EVM**
- Switch to \ref BOOTMODE_QSPI
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

