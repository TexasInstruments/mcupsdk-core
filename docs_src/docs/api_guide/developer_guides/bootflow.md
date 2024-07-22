# Understanding the bootflow and bootloaders {#BOOTFLOW_GUIDE}

[TOC]

## Introduction

This section is aimed at explaining what a bootloader is, why is it needed and details regarding the
bootloader and bootflow used in the MCU+SDK.

### What Is Bootloader?

Generally speaking, bootloader is a piece of software / firmware that runs as soon as you power on an SoC. A bootloader's main
duty is to start the subsequent pieces of software, such an OS, a baremetal application or in some cases another bootloader.
When it comes to embedded, the bootloader is usually closely tied with the underlying SoC architecture. The bootloader is
usually stored in a protected, non-volatile on-chip memory. Typically the bootloader performs various hardware checks,
initializes the processor and configures the SoC registers etc. Since the primary aim of the bootloader is to load the
next piece of software, it needs to communicate to the external world to receive this firmware/application image. This
can be over a number of protocols - USB, UART, SPI, I2C, External Flash, Internal Flash, SD Card, Ethernet, CAN etc.
Bootloader is also a key component in embedded security. Hardware Root-of-Trust is usually passed onto the bootloader and
is passed on down the line.

\note In devices supported by MCU+SDK, the first stage bootloader is burned into read-only memory of the device and is
considered as device firmware / ROM. Mostly in MCU+SDK when we say "bootloader" we are referring to the secondary bootloader,
or the SBL

### Multi-Stage Bootloader

As mentioned earlier, sometimes the bootloader would be loading another bootloader which can load another one and so on.
This is usually the case in advanced SoCs, and there are various reasons for this. The first bootloader is usually kept
on secure, read-only memory for security reasons and to have a known state always before application software starts.
It makes sense then to keep this bootloader simple and let it do only the bare minimum configuration required. This
secondary bootloader can be complex and configurable to suit the needs of the application. This can also be updated easily
compared to the first stage bootloader. In fact for the devices in MCU+SDK, we have a two-stage bootloading - the first
stage bootloader is called ROM Bootloader (RBL) and the second stage bootloader is called Secondary Bootloader (SBL).

### Multi-Core Bootloading

Multi-core bootloading is almost always a follow up when there is a multi-stage bootloading. In this case the first bootloader
technically might not be even aware of the other cores, it will just boot the next stage bootloader and this second stage
bootloader will take care of the complex bootloading on the different cores.

Loading a multi-core application is slightly more complicated than loading a single core application. There would be concerns
regarding image preparation, shared memory access, and so on. Mostly there would be a particular format in which the individual
core images are created, and then they may/may not be concatenated into a single image for SBL to load. Whatever format the
application image is in, the SBL should be aware of it so that it can parse and load the image correctly.


## Bootloading in MCU+SDK

In the MCU+SDK, the bootflow takes place mainly in two steps after you power ON the device
  - **ROM boot**, in which the ROM bootloader boots a secondary bootloader or an SBL
  - **SBL boot** in which the secondary bootloader boots the application

### ROM Boot

The **RBL** or ROM Bootloader is stored in read-only memory and is almost considered as part of the SoC. The details
regarding the RBL and ROM Boot is out of scope for this user guide. Please refer to the Technical Reference Manual of
the device for more details. But basically the ROM expects an x509 signed binary image of the secondary bootloader to be
provided for boot.

- As soon as the board is powered ON, the ROM bootloader or RBL starts running. The RBL is the primary bootloader.
- Depending on which boot mode is selected, the RBL will load the **secondary bootloader** or SBL from a boot
  media (OSPI flash, SD card or via UART).
- Rest of the booting is done by the SBL.
- The RBL expects the image it boots (SBL in our case) to always be signed as mentioned above. Refer \ref TOOLS_BOOT for
  more information on signing scripts.

The x509 template for ROM looks something like this:

\cond SOC_AM64X || SOC_AM243X
\code
[ req ]
distinguished_name     = req_distinguished_name
x509_extensions        = v3_ca
prompt                 = no

dirstring_type = nobmp

[ req_distinguished_name ]
C                      = US
ST                     = SC
L                      = New York
O                      = Texas Instruments., Inc.
OU                     = DSP
CN                     = Albert
emailAddress           = Albert@gt.ti.com

[ v3_ca ]
basicConstraints = CA:true
1.3.6.1.4.1.294.1.3 = ASN1:SEQUENCE:swrv
1.3.6.1.4.1.294.1.9 = ASN1:SEQUENCE:ext_boot_info

[ swrv ]
swrv = INTEGER:1

[ ext_boot_info ]
extImgSize = INTEGER:0
numComp = INTEGER:3
sbl = SEQUENCE:sbl
fw = SEQUENCE:sysfw
bd2 = SEQUENCE:boardcfg

[ sbl ]

compType = INTEGER:1
bootCore = INTEGER:16
compOpts = INTEGER:0
destAddr = FORMAT:HEX,OCT:0000
compSize = INTEGER:0
shaType = OID:SHAOID
shaValue = FORMAT:HEX,OCT:0000

[ sysfw ]

compType = INTEGER:2
bootCore = INTEGER:0
compOpts = INTEGER:0
destAddr = FORMAT:HEX,OCT:0000
compSize = INTEGER:0000
shaType = OID:SHAOID
shaValue = FORMAT:HEX,OCT:0000

{SYSFW_INNER_CERT_SEQ}

[ boardcfg ]

compType = INTEGER:18
bootCore = INTEGER:0
compOpts = INTEGER:0
destAddr = FORMAT:HEX,OCT:0000
compSize = INTEGER:0000
shaType = OID:SHAOID
shaValue = FORMAT:HEX,OCT:0000
\endcode
\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM273X || SOC_AWR294X || SOC_AM261X
\code
[ req ]
distinguished_name     = req_distinguished_name
x509_extensions        = v3_ca
prompt                 = no

dirstring_type = nobmp

[ req_distinguished_name ]
C                      = US
ST                     = SC
L                      = New York
O                      = Texas Instruments., Inc.
OU                     = SITARA MCU
CN                     = Albert
emailAddress           = Albert@gt.ti.com

[ v3_ca ]
basicConstraints = CA:true
1.3.6.1.4.1.294.1.1=ASN1:SEQUENCE:boot_seq
1.3.6.1.4.1.294.1.2=ASN1:SEQUENCE:image_integrity
1.3.6.1.4.1.294.1.3=ASN1:SEQUENCE:swrv

[ boot_seq ]
certType     =  INTEGER:1
bootCore     =  INTEGER:16
bootCoreOpts =  INTEGER:0
destAddr     =  FORMAT:HEX,OCT:70002000
imageSize    =  INTEGER:199488

[ image_integrity ]
shaType = OID:2.16.840.1.101.3.4.2.3
shaValue = FORMAT:HEX,OCT:1d1b24e6487709f007d87c8b2b593abf1853a82a99a54650de85f40ddc7b5ae4558a68e49ea3668732ea34ff4bcf76cc73e4e354a3b8128726843c71b05c4168

[ swrv ]
swrv = INTEGER:1

\endcode
\endcond

Depending on the device type, these are the validation requirements for ROM:

\cond SOC_AM64X || SOC_AM243X || SOC_AM273X || SOC_AWR294X
\imageStyle{device_types_validation_req.png,width:50%}
\image html device_types_validation_req.png "Validation for Device Types"
\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
\imageStyle{device_types_validation_req_am263x.png,width:50%}
\image html device_types_validation_req_am263x.png "Validation for Device Types"
\endcond

\cond SOC_AM64X || SOC_AM243X
#### System Controller Firmware (SYSFW)
- In case of @VAR_SOC_NAME, there is one more component called the System Controller Firmware (SYSFW) which is
  important in the booting process and SoC operation. As the name suggests it is controller firmware which runs in the
  Cortex M3 core and acts as a centralized server for the SoC which grants services in

  - Resource Management (RM)
  - Power Management (PM)
  - Security

- Since the services provided by SYSFW are fundamental, it needs to be loaded before the secondary bootloader can do pretty
  much anything.
- In MCU+SDK we follow what's called the combined bootflow (for more details on combined bootflow refer \ref BOOTFLOW_MIGRATION_GUIDE),
  where RBL boots the SBL and the SYSFW. For this we have to prepare the boot image for RBL specially. It will be a
  concatenation of the SBL binary, SYSFW binary and Board Configuration binary all signed with a single x509 certificate.
  This process is taken care in SBL makefiles, so in most cases the user need not worry about it. For details regarding
  the signing script used for this please refer \ref TOOLS_BOOT
- One of the first things SBL is going to do is wait for a boot notification from SYSFW.
- There are different types of SYSFW images included in the SDK. You can find below binary files under `source/drivers/sciclient/soc/${soc}`:
  - `sysfw-hs-fs-enc.bin`: This is the encrypted and signed SYSFW binary to be packaged with SBL when preparing the SBL
    boot image for **HS-FS** device.
  - `sysfw-hs-fs-enc-cert.bin`: This is the x509 certificate binary generated while signing the above (`sysfw-hs-fs-enc.bin`) image.
    This is also to be packaged with SBL when preparing the SBL boot image for **HS-FS** device.
  - `sysfw-hs-enc.bin`: This is the encrypted and signed SYSFW binary to be packaged with SBL when preparing the SBL
    boot image for **HS-SE**  device.
  - `sysfw-hs-enc-cert.bin`: This is the x509 certificate binary generated while signing the above (`sysfw-hs-enc.bin`) image.
    This is also to be packaged with SBL when preparing the SBL boot image for **HS-SE** device.

- For more information regarding the SYSFW please refer to the TISCI Documentation : https://software-dl.ti.com/tisci/esd/latest/index.html
\endcond
- For ROM to accept any image to boot, there are some restrictions in the image preparation

#### Preparing the SBL for boot

The SBL is like any other application, created using the same compiler and linker toolchain. It is an example implementation
rather than a deliverable. It is customizable by users, but must adhere to the requirements by RBL which is a constant as mentioned above.
However the steps to convert the application `.out` into a bootable image are different for SBL as listed below:

\cond SOC_AM64X || SOC_AM243X || SOC_AM263X || SOC_AM263PX || SOC_AM273X || SOC_AWR294X || SOC_AM261X
\cond ~SOC_AWR294X
- The SBL entry point needs to be different vs other applications. On @VAR_SOC_NAME after power-ON ROM boots the SBL and sets the entry point of SBL to
  both R5FSS0-0 as well as R5FSS0-1. However for SBL we need to detect the core and run SBL only on Core0 and keep Core1 in `wfi` loop.
  This is done by specifying a different entry point `-e_vectors_sbl` in the linker command file for the SBL application. In `_vectors_sbl` the very first thing it does is detect the core and continue execution for Core0, while if the core is Core1 then it enters `wfi` loop.
\endcond
\cond SOC_AWR294X
- The SBL entry point needs to be different vs other applications. On @VAR_SOC_NAME after power-ON ROM boots the SBL and sets the entry point of SBL to R5FSS0-0.
\endcond
- Other special factors for SBL application are listed below
\cond SOC_AM64X || SOC_AM243X
  - After entering `main()`, make sure to call `Bootloader_socWaitForFWBoot` to wait for the boot notification from the SYSFW
  - The linker command file for SBL has to place vectors at address `0x70000000` and this is the entry point for the SBL.
  - Nothing should be placed in ATCM or BTCM
  - Only the region `0x70000000` to `0x70080000` should be used by SBL code, data, stack etc
\endcond
\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
  - The linker command file for SBL has to place vectors at address `0x70002000` and this is the entry point for the SBL.
  - Nothing should be placed in ATCM or BTCM
  - Currently, the region 0x70002000 to 0x70040000 is used by the SBL code, data, stack, etc.
\endcond
\cond SOC_AM273X || SOC_AWR294X
  - The linker command file for SBL has to place vectors at address `0x10200000` and this is the entry point for the SBL.
  - Currently, the region `0x10200000` to `0x10220000` is used by the SBL code, data, stack, etc.
\endcond
\endcond
- After building, the SBL application `.out` file is first converted to a binary format `.bin` using the TI ARM CLANG
  `objcopy` tool.
  - This copies the loadable sections from the .out into a binary image stripping all symbol and section information.
  - If there are two loadable sections in the image which are not contiguous then `objcopy` fills the gaps with `0x00`.
  - It is highly recommended to keep all loadable sections together within a SBL application.
\cond SOC_AM64X || SOC_AM243X || SOC_AM263X || SOC_AM263PX || SOC_AM273X || SOC_AWR294X || SOC_AM261X
- This `.bin` file is then signed using the \ref TOOLS_BOOT_SIGNING to create the final `.tiimage` bootable image.
   - The `.tiimage` file extension is kept to separate the SBL boot image from a normal application image
   - The rom_degenerateKey.pem is used for this.
   - This is a ROM bootloader requirement and is needed even on a non-secure device.
   - The signing tools take the `.bin` file
\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
- The SBL communicates with ROM to get the HSMRt (TIFS-MCU) loaded on the HSM. This firmware provides various foundational security services. For HS-FS ,this can be found at source/drivers/hsmclient/soc/am263x/hsmRtImg.h. For HS-SE devices, more information can be found at MySecureSW. For integrating HSM RunTime with SBL, the following should be taken care of:
   - HSMClient should be initialized and registered.
   - HSMRT (TIFS-MCU) image signed appropriately should be available. For HS-FS, this is already part of the SDK, for HS-SE, this can be compiled with TIFS-MCU package (available on MySecureSW)
\endcond
\cond SOC_AM273X || SOC_AWR294X
- The SBL communicates with ROM to get the HSMRt (TIFS-MCU) loaded on the HSM on HS-SE devices. This firmware provides various foundational security services. More information can be found at MySecureSW. Support for HS-FS will be provided in upcoming releases. For integrating HSM RunTime with SBL, the following should be taken care of:
   - HSMClient should be initialized and registered.
   - HSMRT (TIFS-MCU) image signed appropriately should be available and this is already part of the SDK.
\endcond
- Depending on the device type for which we build the SBL, there will be certain prefixes to the `.tiimage` extension like so:
\cond SOC_AM64X || SOC_AM243X
  - **GP** device:
    - `sbl_xxx.release.tiimage` [No prefix before `.tiimage`, plain image]
  - **HS-FS** device:
    - `sbl_xxx.release.hs_fs.tiimage` [`hs_fs` prefix before `.tiimage`]
\endcond
\cond SOC_AM263X || SOC_AM263PX || SOC_AM273X || SOC_AWR294X || SOC_AM261X
  - **HS-FS** device:
    - `sbl_xxx.release.tiimage` [No prefix before `.tiimage`, plain image]
\endcond
  - **HS-SE** device:
    - `sbl_xxx.release.hs.tiimage` [`hs` prefix before `.tiimage`]
\cond SOC_AM64X || SOC_AM243X
- Note that if we just mentioned `hs` it is meant for **HS-SE** device and `hs_fs` or `hs-fs` is meant for **HS-FS** device.
\endcond
\cond SOC_AM263X || SOC_AM263PX || SOC_AM273X || SOC_AWR294X || SOC_AM261X
- Note that if we just mentioned `hs` it is meant for **HS-SE** device.
\endcond
- The `.tiimage` file can then be flashed or copied to a boot image using the \ref TOOLS_FLASH

\cond SOC_AM64X || SOC_AM243X
- In case of AM243x/AM64x we use combined boot flow, so the tiimage will have SYSFW and BoardCfg as additional components
  as ROM is going to boot these as well. As you can see from the image the signing is done for a concatenated binary of all three.
  For more details on combined boot flow please refer to \ref BOOTFLOW_MIGRATION_GUIDE

\imageStyle{tiimage_k3.png,width:20%}
\image html tiimage_k3.png "TIIMAGE"
\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM273X || SOC_AWR294X || SOC_AM261X
\imageStyle{tiimage_normal.png,width:20%}
\image html tiimage_normal.png "TIIMAGE"
\endcond

\endcond

### SBL Boot

- An SBL typically does a bunch of SOC specific initializations and proceeds to the application loading.
- Depending on the type of SBL loaded, SBL looks for the **multicore appimage** (refer \ref TOOLS_BOOT for more on multicore appimage)
  of the application binary at a specified location in a boot media.
- If the appimage is found, the multicore appimage is parsed into multiple **RPRCs**. These are optimized binaries which
  are then loaded into individual CPUs.
- Each RPRC image will have information regarding the core on which it is to be loaded, entry points and multiple sections
  of that application binary
- The SBL uses this information to initialize each core which has a valid RPRC. It then loads the RPRC according to the
  sections specified, sets the entry points and releases the core from reset. Now the core will start running.

\inlineVideo{sbl_boot.mp4,SBL BOOT,width=50%}

\cond SOC_AM64X || SOC_AM243X || SOC_AM263PX || SOC_AM261X
- To understand the steps to use XIP, see \subpage BOOTFLOW_XIP
\endcond

- Now, as mentioned above, to boot an application with SBL it has to be specially prepared after it's compiled.

#### Preparing the application for boot

\note To see the exact sequence of steps in which applications and secondary bootloader (SBL) are converted from compiler generated .out files to
      boot images, see the makefile `makefile_ccs_bootimage_gen` that is included in every example and secondary bootloader (SBL) CCS project.

\note If you are using makefile based build, then see the file named `makefile` in the example folder.

Shown below are the different steps that are done to convert the compiler+linker generated application `.out` into a format suitable for flashing
and booting

\cond SOC_AM243X || SOC_AM64X
- For each CPU, the compiler+linker toolchain is used to create the application .out "ELF" file which can be loaded and run via CCS
- The below "post build" steps are then used to convert the application .out into a "flash" friendly format
  - For each CPU, `out2rpc` is used to convert the ELF .out to a binary file containing only the loadable sections. This is called a RPRC file.
  - For each CPU, `xipGen` is used to split this RPRC file into two RPRC files.
     - One RPRC, containing the section that during boot need to be loaded to RAM
     - Second RPRC, containing the section that during boot are not loaded to RAM but are instead "eXecuted In Place", i.e XIP
  - `multiCoreGen` is then used to combine all the non-XIP RPRC files per CPU into a single `.appimage` file which is a concatenation of the
     individual CPU specific RPRC files.
  - `multiCoreGen` is used again to combine all the XIP RPRC files per CPU into a single `.appimage_xip` file which is a concatenation of the
     individual CPU specific RPRC XIP files.
- This `.appimage` and `.appimage_xip` is then flashed to the board

\imageStyle{bootflow_post_build_steps.png,width:50%}
\image html bootflow_post_build_steps.png "Post build steps"
\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM273X || SOC_AWR294X || SOC_AM261X
- For each CPU, the compiler+linker toolchain is used to create the application .out "ELF" file which can be loaded and run via CCS
- The below "post build" steps are then used to convert the application .out into a "flash" friendly format
  - For each CPU, `out2rpc` is used to convert the ELF .out to a binary file containing only the loadable sections. This is called a RPRC file.
  - `multiCoreGen` is then used to combine all the RPRC files per CPU into a single `.appimage` file which is a concatenation of the
     individual CPU specific RPRC files.
- This `.appimage` is then flash to the board.

\imageStyle{bootflow_post_build_steps_no_xip.png,width:50%}
\image html bootflow_post_build_steps_no_xip.png "Post build steps"
\endcond

#### Flashing the application for boot

\cond SOC_AM243X || SOC_AM64X
- Once the application images (`.appimage` and `.appimage_xip`) are created one needs to copy or flash these
  to a supported boot media so that the application can start executing once the SOC is powered ON
\endcond
\cond SOC_AM263X || SOC_AM263PX || SOC_AM273X || SOC_AWR294X || SOC_AM261X
- Once the application images (`.appimage`) is created one needs to copy or flash these
  to a supported boot media so that the application can start executing once the SOC is powered ON
\endcond
- When flashing the application we also need to flash a bootloader or SBL image.
- See \ref TOOLS_FLASH for detailed steps that are done to flash a user application

#### Booting the application

After a SBL and application image is flashed, shown below is the high level boot flow, after the SOC is powered on.

\imageStyle{bootflow_main.png,width:40%}
\image html bootflow_main.png "HIGH LEVEL BOOTFLOW"

\cond SOC_AM243X || SOC_AM64X || SOC_AM263X || SOC_AM263PX || SOC_AM273X || SOC_AWR294X || SOC_AM261X
## Secure Boot

In secure device variants, there are slight differences in the bootflow. For details on secure boot, please refer \ref SECURE_BOOT
\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
## PBIST For 200MHz and 400MHz R5F Core Variants

pBIST (parallel Built-In Self-Test) can be performed for both 200MHz and 400MHz part numbers by including the instance of `mcu_bist` in the sys-config for the bootloader examples. By default it's enabled only for bootloader `sbl-sd`. For 400 MHz part numbers ROM performs pBIST on R5FSS0 Memories and L2 Memory - Bank-0 and Bank-1. But due to ROM limitation this test is not done by ROM for 200 MHz part numbers. The `mcu_bist` instance can be added or removed on the basis of use-case. pBIST is perfromed only on L2 memory - three banks (Bank-1,Bank-2,Bank3 (not on Bank-0)) and R5FSS1 TCM Memories. Please refer below for more information.

  <p>\note Important Information:<br>
    - pBIST is a destructive test and cannot be run on active memories.<br>
    - Perform pBIST on specific sections of RAM and Subsystem 1 TCM, excluding active areas.<br>
    - SBL code must resides in the 0th bank of L2 memory to perform pBIST on the other three banks, as `mcu_bist` instance will run pBIST on L2 memory - Bank 1, Bank 2, Bank 3. The `mcu_bist` instance can be added for `sbl_qspi` as it resides in Bank-0. If SBL resides in banks other than Bank-0, the pBIST setup must be updated with the specific memories.<br>
  </p>

<table>
    <tr>
      <th>Step</th>
      <th>Description</th>
    </tr>
    <tr>
      <td>1</td>
      <td>SBL Startup<br>
        <ul>
          <li>Ensure SBL is installed correctly on the target device</li>
          <li>Power on or reset the device to initiate SBL startup</li>
        </ul>
      </td>
    </tr>
    <tr>
      <td>2</td>
      <td>Efuse Bit Detection<br>
        <ul>
          <li>During SBL startup, efuse bits will be checked</li>
          <li>If 200 MHz variant is detected, proceed to Step 3</li>
          <li>Otherwise, skip to the next startup process</li>
        </ul>
      </td>
    </tr>
    <tr>
      <td>3</td>
      <td>pBIST Execution<br>
        <ul>
          <li>Initiating pBIST if 200 MHz variant is detected</li>
          <li>pBIST performed on specific sections of RAM and Subsystem 1 TCM memories, excluding active areas i.e., Subsystem 0 TCM memories</li>
          <li>SBL code resides in the 1st bank of L2 memory (Bank-0); So pBIST is performed on the other three banks of L2 memories.</li>
        </ul>
      </td>
    </tr>
  </table>

 In software workaround, pBIST is perfromed only on L2 memory - three banks (Bank-1,Bank-2,Bank3 (and not on Bank-0)) and R5FSS1 TCM Memories. \n
 User can perfrom pBIST on the remaining L2 Memory bank-0 and R5FSS0 TCM memories on their own by following the steps given below:
 The following table provides an overview of the steps to perform pBIST on RFSS0 TCM and L2 Memory Bank-0:

  <table>
  <tr>
  <th>Step</th>
  <th>Description</th>
  </tr>
  <tr>
  <td>1</td>
  <td>Use the SDL example of pBIST that utilizes the SDL pBIST library.</td>
  </tr>
  <tr>
  <td>2</td>
  <td>Set up the test environment to specify L2 bank-0 and R5FSS0 TCM memories.</td>
  </tr>
  <tr>
  <td>3</td>
  <td>Update the SBL to be loaded on other memory banks (excluding bank 0).</td>
  </tr>
  <tr>
  <td>4</td>
  <td>The SBL will load the application on the RFSS1 core, which will run pBIST on the remaining L2 bank-0 and R5FSS0 TCM memories.</td>
 </tr>
  <tr>
  <td>5</td>
  <td>Once pBIST is completed, the device will be reset to proceed with normal booting.</td>
  </tr>
  </table>

 By following these steps, you can perform pBIST on the specified L2 memory bank-0 and R5FSS0 TCM memories, ensuring proper self-testing, and allowing the device to boot normally afterwards.


•	The pBIST can be run for both 200 MHz and 400 MHz part variants. To run the pBIST user can include `mcu_bist` instance for the specific bootloader in the syscfg. \n
•   By default it's enabled only for bootloader `sbl-null`. \n
•	The software workaround described in this user guide for L2 Memory Bank-0 and R5FSS0 TCM memories is specifically designed to address the ROM errata affecting 200 MHz R5F core variants. \n
•	The pBIST procedure performed during the SBL startup helps mitigate the limitations imposed by the ROM. \n
•	It is important to ensure that the SBL and associated software are correctly installed and configured on the target device for the workaround to function effectively.

To know more about pBIST and overall SDL support for pBIST, please take a look at \ref SDL_PBIST_PAGE
\endcond
## Deep Dive into SBLs

The SBL is like any other example of the SDK. They use the bootloader library APIs to carry out the bootloading process.
Depending on the boot media from which we load the application binary, we have multiple SBLs like `sbl_ospi`,`sbl_uart` etc.
A bare minimum SBL called the `sbl_null` is also included which aids the users to load their applications via CCS. Here are
some details regarding those.

### SBL NULL

- The `sbl_null` is a secondary bootloader which doesn't load any application binary, but just does the SOC initialization and puts all the cores in WFI (Wait For Interrupt) mode.
- This is supposed to be a "development form" bootloader which should be used only during initial development.
- The other method is using NO-BOOT/DEV-BOOT boot modes of the devices and using GEL scripts to initialize the SoC via debugger. The application
  binaries can then be side-loaded. ROM is not involved in this case. The `sbl_null` is an alternative to this process.
\cond SOC_AM64X || SOC_AM243X
- In HS-SE devices, the debugger is usually closed. So using the `sbl_null` is the only option to initialize the device in this case.
\endcond
- This is referred to as the SOC initialization binary, refer \ref EVM_FLASH_SOC_INIT for more on this.

\cond SOC_AM64X || SOC_AM243X
### SBL SD {#BOOTFLOW_SBL_SD}

- The `sbl_sd` is a secondary bootloader which reads the application image file from the SD card and then moves on to core initialization and other steps

- To boot an application using the `sbl_sd`, the application image needs to be copied to the SD card as a file named "app". Make sure that the SD card is formatted to have a FAT partition. To know more about the SD card partitioning please refer \ref EVM_SOC_INIT_SD_BOOT_MODE

- Follow the steps in the above referred page to partition the SD card. For a complete boot from SD card, both the `sbl_sd` binary and the application image binary has to be present as files in the SD card. You have to rename the `sbl_sd` appimage as 'tiboot3.bin'.

        copy file to SD card => ${SDK_INSTALL_PATH}/tools/boot/sbl_prebuilt/@VAR_BOARD_NAME_LOWER/sbl_sd.release.hs_fs.tiimage
        rename in SD card as => tiboot3.bin

- Similarly you can copy any appimage file to the SD card and rename in the SD card as "app" so that the SBL can pick it up.

- Currently the `sbl_sd` reads the full appimage file into an MSRAM buffer and then parses the multicore appimage. Because of this reason **appimages higher than ~380 KB in size can't be booted by `sbl_sd` as of now**.

### SBL OSPI

- The `sbl_ospi` is a secondary bootloader which reads and parses the application image from a location in the OSPI flash and then moves on to core initialization and other steps

- To boot an application using the `sbl_ospi`, the application image needs to be flashed at a particular location in the OSPI flash memory.

- This location or offset is specified in the SysConfig of the `sbl_ospi` application. Currently this is 0x80000. This offset is chosen under the assumption that the `sbl_ospi`
  application takes at max 512 KB from the start of the flash. If a custom bootloader is used, make sure that this offset is chosen in such a way that it is greater than the
  size of the bootloader which is being flashed and also aligns with the block size of the flash.

- To flash an application (or any file in fact) to a location in the OSPI flash memory, follow the steps mentioned in \ref BASIC_STEPS_TO_FLASH_FILES

### SBL OSPI MULTI-PARTITION

- The `sbl_ospi_multipartition` is a secondary bootloader which reads and parses core-specific application images for a system project from pre-defined locations
  in the OSPI flash and then moves on to core initialization and other steps

- To boot an application using the `sbl_ospi_multipartition`, the application images needs to be flashed at specific locations in the OSPI flash memory. The default
  offsets are 512 KB apart.

- This locations or offsets are specified in the SysConfig of the `sbl_ospi_multipartition` application.

- For more information please refer to the example documentation of `sbl_ospi_multipartition` here: \ref EXAMPLES_DRIVERS_SBL_OSPI_MULTI_PARTITION

- To flash an application (or any file in fact) to a location in the OSPI flash memory, follow the steps mentioned in \ref BASIC_STEPS_TO_FLASH_FILES

\endcond

\cond SOC_AM263X || SOC_AM273X || SOC_AWR294X
### SBL QSPI

- The `sbl_qspi` is a secondary bootloader which reads and parses the application image from a location in the QSPI flash and then moves on to core initialization and other steps

- To boot an application using the `sbl_qspi`, the application image needs to be flashed at a particular location in the QSPI flash memory.
\cond ~SOC_AWR294X
- This location or offset is specified in the SysConfig of the `sbl_qspi` application. Currently this is 0x80000. This offset is chosen under the assumption that the `sbl_qspi`
  application takes at max 512 KB from the start of the flash. If a custom bootloader is used, make sure that this offset is chosen in such a way that it is greater than the
  size of the bootloader which is being flashed and also aligns with the block size of the flash.
\endcond
\cond SOC_AWR294X
- This location or offset is specified in the SysConfig of the `sbl_qspi` application. Currently this is 0xA0000. This offset is chosen under the assumption that the `sbl_qspi`
  application takes at max 640 KB from the start of the flash. If a custom bootloader is used, make sure that this offset is chosen in such a way that it is greater than the
  size of the bootloader which is being flashed and also aligns with the block size of the flash.
\endcond
- To flash an application (or any file in fact) to a location in the QSPI flash memory, follow the steps mentioned in \ref BASIC_STEPS_TO_FLASH_FILES

\endcond

\cond SOC_AM263PX || SOC_AM261X
### SBL OSPI

- The `sbl_ospi` is a secondary bootloader which reads and parses the application image from a location in the OSPI flash and then moves on to core initialization and other steps

- To boot an application using the `sbl_ospi`, the application image needs to be flashed at a particular location in the OSPI flash memory.

- This location or offset is specified in the SysConfig of the `sbl_ospi` application. Currently this is 0x80000. This offset is chosen under the assumption that the `sbl_ospi`
  application takes at max 512 KB from the start of the flash. If a custom bootloader is used, make sure that this offset is chosen in such a way that it is greater than the
  size of the bootloader which is being flashed and also aligns with the block size of the flash.

- To flash an application (or any file in fact) to a location in the OSPI flash memory, follow the steps mentioned in \ref BASIC_STEPS_TO_FLASH_FILES

\endcond

### SBL UART

- The `sbl_uart` is a secondary bootloader which receives the multicore appimage via UART, stores it in memory and then does the parsing, core initialization etc.

- To boot an application using the `sbl_uart`, you can refer to \ref UART_BOOTLOADER_PYTHON_SCRIPT subsection. Detailed steps on the usage is mentioned in the same subsection.

\cond SOC_AM64x || SOC_AM243x

### SBL DFU

- The SBL DFU is a secondary bootloader which receives the multicore appimage via USB DFU. It Stores the received appimage in RAM memory and boots the application.

- Refer \ref EXAMPLES_DRIVERS_SBL_DFU to know more about SBL DFU.

- To boot an application using the `sbl_uart`, you can refer to \ref USB_BOOTLOADER subsection. Detailed steps on the usage is mentioned in the same subsection.

\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
### SBL CAN

- The `sbl_can` is a secondary bootloader which needs to be flashed in QSPI Flash.

- The `sbl_can` receives the multicore appimage via CAN, stores it in memory and then does the parsing, core initialization etc.

- To boot an application using the `sbl_can`, you can refer to \ref CAN_BOOTLOADER_PYTHON_SCRIPT subsection.

### SBL SD {#BOOTFLOW_SBL_SD}

- The `sbl_sd` is a secondary bootloader which reads the application image file from the SD card and then moves on to core initialization and other steps

- To boot an application using the `sbl_sd`, the application image needs to be copied to the SD card as a file named "app"

- Similarly you can copy any appimage file to the SD card and rename in the SD card as "app" so that the SBL can pick it up.

- Currently the `sbl_sd` reads the full appimage file into an MSRAM buffer and then parses the multicore appimage. Because of this reason **appimages higher than ~512 KB in size can't be booted by `sbl_sd` as of now**.

- To boot and appication using `sbl_sd`, you can refer to \ref EXAMPLES_DRIVERS_SBL_SD subsection.
\endcond

\cond SOC_AM64X || SOC_AM243X

### SBL PCIE

- The `sbl pcie` is a secondary bootloader which receives the multicore appimage via
PCIe to memory and then does the parsing, core initialization etc.

- The bootloader and appimage for `sbl pcie` is send over to the target board from a host
board running \ref EXAMPLES_DRIVERS_SBL_PCIE_HOST.

- For detailed steps on running `sbl pcie`, you can refer to \ref EXAMPLES_DRIVERS_SBL_PCIE.

\endcond

\cond SOC_AM64X || SOC_AM243X

### SBL EMMC

- The `sbl_emmc` is a secondary bootloader which reads and parses the application image from a location in the eMMC and then moves on to core initialization and other steps.

- To boot an application using the `sbl_emmc`, the application image needs to be flashed at a particular location in the eMMC.

- This location or offset is specified in the SysConfig of the `sbl_emmc` application. Currently this is 0x800000.

- To flash an application (or any file in fact) to a location in the eMMC, follow the steps mentioned in \ref BASIC_STEPS_TO_FLASH_FILES

\endcond

\cond SOC_AM64X
### SBL OSPI LINUX

- The `sbl_ospi_linux` is a secondary bootloader which boots Linux on A53 core and RTOS/NORTOS application on R5, M4 cores.

- To boot Linux and RTOS/NORTOS applications using `sbl_ospi_linux`, the Linux appimage (see \ref LINUX_APPIMAGE_GEN_TOOL) and the RTOS/NORTOS application images needs to be flashed at a particular location in the OSPI flash memory.

- This location or offset is specified in the SysConfig of the `sbl_ospi_linux` application. Currently this is 0x80000 for RTOS/NORTOS images and 0x300000 for Linux application image.

- To flash an application (or any file in fact) to a location in the OSPI flash memory, follow the steps mentioned in \ref BASIC_STEPS_TO_FLASH_FILES

### SBL EMMC LINUX

- The `sbl_emmc_linux` is a secondary bootloader which boots Linux on A53 core and RTOS/NORTOS application on R5, M4 cores from eMMC.

- To boot Linux and RTOS/NORTOS applications using `sbl_emmc_linux`, the Linux appimage (see \ref LINUX_APPIMAGE_GEN_TOOL) and the RTOS/NORTOS application images needs to be flashed at a particular location in the eMMC.

- This location or offset is specified in the SysConfig of the `sbl_emmc_linux` application. Currently this is 0x800000 for RTOS/NORTOS images and 0xA00000 for Linux application image. In most cases you wouldn't need to change this.

- To flash an application (or any file in fact) to a location in the eMMC, follow the steps mentioned in \ref BASIC_STEPS_TO_FLASH_FILES

\endcond

## Additional References

See also these additional pages for more details and examples about the boot flow,

- To understand different secondary bootloader (SBL) examples see,
  - \ref EXAMPLES_DRIVERS_SBL_NULL
  - \ref EXAMPLES_DRIVERS_SBL_UART_UNIFLASH
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
\cond SOC_AM263X
  - \ref EXAMPLES_DRIVERS_SBL_QSPI
  - \ref EXAMPLES_DRIVERS_SBL_UART
  - \ref EXAMPLES_DRIVERS_SBL_CAN
\endcond

\cond SOC_AM263PX || SOC_AM261X
  - \ref EXAMPLES_DRIVERS_SBL_OSPI
  - \ref EXAMPLES_DRIVERS_SBL_UART
  - \ref EXAMPLES_DRIVERS_SBL_CAN
\endcond

- To understand the flashing steps, see \ref TOOLS_FLASH
- To understand the boot image creation tools, see \ref TOOLS_BOOT
\cond SOC_AM64X
- To understand the details on SBL Booting linux from OSPI, see \subpage SBL_BOOTING_LINUX_OSPI
- To understand the details on SBL Booting linux from eMMC, see \subpage SBL_BOOTING_LINUX_EMMC
\endcond

