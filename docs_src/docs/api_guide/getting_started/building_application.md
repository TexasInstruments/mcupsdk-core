# Building an Application {#BUILDING_APPLICATION}

[TOC]

Building an example is converting the source files of the application into a machine readable file. 

- It involves two primary steps:
	- Compilation
	- Linking

- A compiler converts the human readable source code to assembly language, producing assembly files.
- The assembler converts assembly files into machine code, creating object files (.o/.obj)
- These files are not yet linked into an executable.
- A linker.cmd file specifies memory layout and sections placement information. The linker then combines the multiple object files into a single executable. 

## Tools

<table>
<tr>
    <th>Tool
    <th>Reference
    <th>Description
</tr>
<tr>
    <td>Compiler
    <td>\ref INSTALL_TIARMCLANG
    <td>Converts .c source code to machine readable code.
</tr>
<tr>
    <td>Sysconfig
    <td>\ref INSTALL_SYSCONFIG
    <td>A GUI tool used for configuring pins, peripherals, software stacks, RTOS, clock tree, linker (see row below) and other components.
</tr>
<tr>
    <td>Creating linker.cmd file
    <td>\ref MEMORY_CONFIGURATOR
    <td>A GUI method of creating a linker file using Sysconfig tool. It generates a linker.cmd file along with other component files.
</tr>
<tr>
    <td>.out to .rprc
    <td>\ref OUT2RPRC_TOOL
    <td>Converts the application executable (.out) into custom TI RPRC (.rprc) image.
</tr>
<tr>
    <td>.rprc to .appimage
    <td>\ref MULTICOREIMAGEGEN_TOOL
    <td>Converts the RPRC files created for each CPU into a single combined multicore application image that can be booted by the SBL.
</tr>
<tr>
    <td>.out to .mcelf
    <td>\ref MCELF_GEN_TOOL
    <td>Takes individual core .out files as input and combines them to create a single .mcelf ELF file which can be booted by the SBL.
</tr>
<tr>
    <td>Signing tools
    <td>\ref TOOLS_BOOT_SIGNING
    <td>Collection of scripts needed to sign SBL and application images.
</tr>
</table>

## Creation of application binary

\note To see the exact sequence of steps in which applications and secondary bootloader (SBL) are converted from compiler generated .out files to
      boot images, see the makefile `makefile_ccs_bootimage_gen` that is included in every example and secondary bootloader (SBL) CCS project.

\note If you are using makefile based build, then see the file named `makefile` in the example folder.

Shown below are the different steps that are done to convert the compiler+linker generated application `.out` into a format suitable for flashing and booting.

### Generating .out binary

  - For each CPU, the compiler + linker toolchain is used to create a .out file.
  - The compiler convertes the .c files into .asm files, the assembler converts .asm files to object files - .obj files. 
  - The linker then combines the multiple object files into a single .out file for each CPU.

\imageStyle{compiler_build_steps_no_xip.png,width:20%}
\image html compiler_build_steps_no_xip.png "Generating .out file"

  - This .out file can be loaded and run via CCS. Refer \ref CCS_LOAD_RUN

### Generating .appimage binary
  - For each CPU, `out2rpc` is used to convert the ELF .out to a binary file containing only the loadable sections. This is called a RPRC file.
  - `multiCoreGen` is then used to combine all the RPRC files per CPU into a single `.appimage` file which is a concatenation of the
     individual CPU specific RPRC files.
  - This `.appimage` is then flashed to the board.

\imageStyle{bootflow_post_build_steps_no_xip.png,width:50%}
\image html bootflow_post_build_steps_no_xip.png "Post build steps RPRC"

### Generating .mcelf binary
  - Refer \ref MCELF_LANDING for information on MCELF
  - The mcelf image generator script `genimage.py` takes each individual core's .out file as input and combines them to form a .mcelf file.
  - This .mcelf file contains metadata and segments along with information like segment type, load address, size, alignment.
  - The `.mcelf` file is then flashed to the board.

\imageStyle{mcelf_bootflow_post_build_steps_no_xip.png,width:50%}
\image html mcelf_bootflow_post_build_steps_no_xip.png "Post build steps MCELF"

### Signing the binary

Image			            |	HSFS			|	HSSE
----------------------|-----------|--------------
SBL is signed				  |	Yes		    |	Yes
Application	is signed	|	No		    |	Yes

#### SBL Signing {#SBL_SIGNING}

- After building, the SBL application `.out` file is first converted to a binary format `.bin` using the TI ARM CLANG `objcopy` tool.
- This `.bin` file is then signed using the \ref TOOLS_BOOT_SIGNING to create the final `.tiimage` bootable image.
   - The `.tiimage` file extension is kept to separate the SBL boot image from a normal application image
   - The rom_degenerateKey.pem is used for this.
   - This is a ROM bootloader requirement and is needed even on a non-secure device.
   - The signing tools take the `.bin` file

- The SBL communicates with ROM to get the HSMRt (TIFS-MCU) loaded on the HSM. This firmware provides various foundational security services. For HS-FS ,this can be found at source/drivers/hsmclient/soc/am263x/hsmRtImg.h. For HS-SE devices, more information can be found at MySecureSW. For integrating HSM RunTime with SBL, the following should be taken care of:
   - HSMClient should be initialized and registered.
   - HSMRT (TIFS-MCU) image signed appropriately should be available. For HS-FS, this is already part of the SDK, for HS-SE, this can be compiled with TIFS-MCU package (available on MySecureSW)

- Depending on the device type for which we build the SBL, there will be certain prefixes to the `.tiimage` extension like so:

  - **HS-FS** device:
    - `sbl_xxx.release.tiimage` [No prefix before `.tiimage`, plain image]

  - **HS-SE** device:
    - `sbl_xxx.release.hs.tiimage` [`hs` prefix before `.tiimage`]

- Note that if we just mentioned `hs` it is meant for **HS-SE** device.

- The `.tiimage` file can then be flashed or copied to a boot image using the \ref TOOLS_FLASH

\imageStyle{tiimage_normal.png,width:15%}
\image html tiimage_normal.png "TIIMAGE"

- For detailed information refer \ref SBL_SECURE_IMAGE

#### Application signing

- Application image is signed only for HSSE secure boot. 
- Refer \ref APPLICATION_SECURE_IMAGE for detailed information.

## Building a Hello world example

Steps to build the hello world example from SDK is mentioned here \ref GETTING_STARTED_BUILD

## Next step - Loading the application
Go to \ref LOADING_APPLICATION