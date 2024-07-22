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


## Signing Scripts {#TOOLS_BOOT_SIGNING}

- To run these scripts, one needs `openssl` installed as mentioned here, \ref INSTALL_OPENSSL
- Signing scripts are a collection of scripts needed to sign ROM images (image booted by ROM - mostly the SBL) and application images (image booted by the SBL)
\cond SOC_AM263X || SOC_AM263PX||SOC_AM273X||SOC_AWR294X || SOC_AM261X
### Signing SBL
\endcond
- The RBL requires the boot image (mostly SBL), to be signed always, even if we are not using secure boot.
  \cond SOC_AM64X || SOC_AM243X
- We follow a combined boot method for ROM images. Here the ROM Bootloader (RBL) boots the SBL, SYSFW and BOARDCFG together. The boot image would be a binary concatenation of x509 Certificate, SBL, SYSFW, BOARDCFG (and the SYSFW inner certificate in case of HS device) binary blobs. We use a python script to generate this final boot image. This script has a dependency on `openssl` as mentioned before, so make sure you've installed it. To generate a combined boot image, one can do as below:

- For GP devices
  \code
  cd ${SDK_INSTALL_PATH}/source/security/security_common/tools/boot/signing
  ${PYTHON} rom_image_gen.py --swrv 1 --sbl-bin <path-to-sbl-binary> --sysfw-bin <path-to-sysfw-binary> --boardcfg-blob <path-to-boardcfg-binary-blob> --sbl-loadaddr ${SBL_RUN_ADDRESS} --sysfw-loadaddr ${SYSFW_LOAD_ADDR} --bcfg-loadaddr ${BOARDCFG_LOAD_ADDR} --key ${BOOTIMAGE_CERT_KEY} --rom-image <path-to-output-image>
  \endcode

- For HS devices, we have to pass the HS SYSFW binaries and also the SYSFW inner certificate to the signing script.
  \code
  cd ${SDK_INSTALL_PATH}/source/security/security_common/tools/boot/signing
  ${PYTHON} rom_image_gen.py --swrv 1 --sbl-bin <path-to-sbl-binary> --sysfw-bin <path-to-sysfw-binary> --sysfw-inner-cert <path-to-sysfw-inner-cert-binary> --boardcfg-blob <path-to-boardcfg-binary-blob> --sbl-loadaddr ${SBL_RUN_ADDRESS} --sysfw-loadaddr ${SYSFW_LOAD_ADDR} --bcfg-loadaddr ${BOARDCFG_LOAD_ADDR} --key ${BOOTIMAGE_CERT_KEY} --debug DBG_FULL_ENABLE --rom-image <path-to-output-image>
  \endcode

- By default SBLs provided in SDK are signed with full debug enable since this is needed for development. You can see from `--debug` switch used above. Once moved to production please remove this switch from the makefile.

- For SBL images or examples which is loaded by SBL, we use a different signing script. This is solely because of the x509 certificate template differences between ROM and SYSFW. In GP devices appimages are not signed. The signing happens only in HS devices. The script usage is:
  \code
  cd ${SDK_INSTALL_PATH}/source/security/security_common/tools/boot/signing
  $(PYTHON) appimage_x509_cert_gen.py --bin <path-to-the-binary> --authtype 1 --key <signing-key-derived-from-devconfig> --output <output-image-name>
  \endcode

- In the case of encryption, two extra options are also passed to the script like so:
  \code
  cd ${SDK_INSTALL_PATH}/source/security/security_common/tools/boot/signing
  $(PYTHON) appimage_x509_cert_gen.py --bin <path-to-the-binary> --authtype 1 --key <signing-key-derived-from-devconfig> --enc y --enckey <encryption-key-derived-from-devconfig> --output <output-image-name>
  \endcode

- These scripts are invoked in makefiles, and the image generation happens automatically along with the example build. So mostly these scripts need not be manually run.
\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM273X || SOC_AWR294X || SOC_AM261X

For SBL images, the script `/source/security/security_common/tools/boot/signing/mcu_rom_image_gen.py`.

For HS-FS devices:
\code
cd ${SDK_INSTALL_PATH}/source/security/security_common/tools/boot/signing
$(PYTHON) mcu_rom_image_gen.py --image-bin <path-to-the-binary> --core R5 --swrv 1 --loadaddr $(SBL_RUN_ADDRESS) --sign-key <signing-key-configured-in-devconfig> --out-image <output-image-name> --debug $(DEBUG_OPTION)
\endcode

For HS-SE devices:
\code
cd ${SDK_INSTALL_PATH}/source/security/security_common/tools/boot/signing
$(PYTHON) mcu_rom_image_gen.py --sbl-enc --enc-key <encryption-key-configured-in-devconfig> --image-bin <path-to-the-binary> --core R5 --swrv 1 --loadaddr $(SBL_RUN_ADDRESS) --sign-key <signing-key-configured-in-devconfig> --kd-salt <path-to-key-derivation-salt> --out-image <output-image-name> --debug $(DEBUG_OPTION)
\endcode

Here, we deep dive into the extension supported in SBL and HSM Runtime (TIFS-MCU)
certificate by ROM.
To get access to the HSM Runtime code and ability to build it for HS-SE device,
please contact your TI represntative to get access to the TIFS-MCU package from
MySecureSW.

#### Boot Information OID (1.3.6.1.4.1.294.1.1)
\code
bootInfo ::=  SEQUENCE {
	cert_type:  INTEGER,		-- identifies the certificate type
	boot_core:	INTEGER,		-- identifies the boot core
	core_opts:	INTEGER,		-- Core Options
	load_addr:	OCTET STRING,	-- Global address image destination
	image_size:	INTEGER,		-- Image size in bytes
}
\endcode
The Boot Information Object identifier provides information about the image
which is being loaded. This information is mandatory and needs to be present
else the image boot will fail.

##### Elements of the extension:
**Certificate Type**: The certificate type defines the type of the image which is
being loaded by the Boot-ROM.

These are the supported values:
- 0x1	R5 SBL Boot Image
- 0x2	HSM Runtime (TIFS-MCU) Image

**Boot core**: The boot core identifies the core on which the image will be executing.

These are the supported values:
- 0x0	HSM Core
- 0x10	R5 Core

**Core Options**: The core options are documented in the table below.

These are the supported values:
- 0x0	    Lock Step Mode
- Non-Zero	Dual Core Mode

\note Currently, configuring this option is not supported with Signing scripts.
This feature will be added in future releases.

Core options are applicable only for the R5 SBL Images.

**Load Address**: The load address will be address in the system where the
 image will be loaded. This is applicable for both SBL and TIFS-MCU (HSMRt)
images.

**Image Size**: This is the size in bytes of the R5 SBL or HSM Runtime Image
to which the certificate has been attached.

#### Image Integrity OID (1.3.6.1.4.1.294.1.2)

\code
imageIntegrity ::= SEQUENCE {
	sha_type:	OID,			-- Identifies the SHA type
	hash:		OCTET STRING	-- The SHA of the boot image
}
\endcode

If the X.509 certificate provides the image integrity boot extension,
the Boot-ROM will perform the SHA-512 on the entire image and will verify
 the computed hash with the hash provided in the boot extension.
 In the case of a mismatch the boot will fail.

##### Elements of the extension:

**SHA Type**: The Boot-ROM only supports SHA-512.

The following values are supported:

2.16.840.1.101.3.4.2.3	SHA-512 Object Identifier

Please refer to the Section 2.4 of the RFC-5754 for the SHA-512 Object Identifier.

**Hash**: This is SHA-512 hash which is calculated over the image (R5 SBL/HSM Runtime)

R5 SBL Image:
-	HS-SE Device: Image Integrity is mandatory
-	HS-FS Device: Image Integrity is optional

HSM Runtime Image:
-	HS-SE Device: Image Integrity is mandatory
-	HS-FS Device: Image Integrity is mandatory

#### Software Revision OID (1.3.6.1.4.1.294.1.3)

\code
softwareRevision ::= SEQUENCE {
   revision:	INTEGER -- Software revision
}
\endcode

The information in the software revision is used to indicate the version of
the image which is being loaded.

##### Elements of the extension:

**revision**:
This is the version number. This will be matched to the EFUSE programmed
version to indicate if the image loading should be done or not. This is
applicable only for SE devices.

|EFUSE Revision|	Certificate Revision|	Description |
|--------------|------------------------|---------------|
|0 |	0 | Ignore the revision checking. Images will *always* be loaded|
|0 | >0 | Device does not mandate revision checking. Images will be loaded|
|>0|	0 |EFUSE Version > Certificate Version. Image will *never* be loaded.|
|>0|	>0 |Image will be loaded only if the Certificate revision >= EFUSE revision|

The following fields in efuse ROM help support the above feature:
- SWRV_SBL- This is used to perform the revision checking while loading the R5 SBL
- SWRV_HSM- This is used to perform the revision checking while loading the HSM Runtime

For more information, about how to fuse in these values, please refer to OTP
Keywriter Documentation available on MySecureSW.

#### Image Encryption OID (1.3.6.1.4.1.294.1.4)

\code
imageEncryption ::= SEQUENCE {
   iv:		OCTET STRING -- The initialization vector
   rs:	      OCTET STRING -- Random string
   iter:		INTEGER      -- Iteration count
   salt:		OCTET STRING -- encryption salt value
}
\endcode

The Boot-ROM only supports AES-CBC mode with 256bit keys. The information in the image encryption object identifier is used to decrypt the image.

##### Elements of the extension:

**IV**:
The initialization vector is used during the AES-CBC decryption procedure. The initialization vector needs to be 16bytes.

**rs**:
This is the random string which is 32bytes long and is appended by the script
at the end of the image. The Boot-ROM will decrypt the image and will perform
 a random string comparison to determine if the decryption was successful.

**iter**:
Iteration Count which is used to determine if the HKDF needs to be performed
and key derivation needs to be done. If the iteration count is 0 then the key
from the e-fuse is used as is for the decryption. If the iteration count is
non-zero then the Boot-ROM will perform the HKDF key derivation using the salt
with exactly one iteration (even if iteration count is > 1).
The derived key is then used for the decryption operation.

**salt**:
The salt is used only if the iteration count is non-zero and key derivation is
being done. The salt is fed to the HKDF module to derive the key. The salt
fields should be 32bytes.

For recommendation on the salt, please refer to \ref SECURE_BOOT.

\note It is recommended to update key derivation salt everytime there is a
firmware upgrade. SBL, HSMRt and application images need to be prepared
again, once the salt is changed.

- HS-SE device: Image Encryption is optional
- HS-FS device: Image Encryption is optional

#### Derivation OID (1.3.6.1.4.1.294.1.5)

\code
derivationKey ::= SEQUENCE {
   salt:      OCTET STRING -- encryption salt value
}
\endcode
The Boot-ROM will leave a derived key in the assets interface for the
TIFS-MCU. The key is derived using HKDF from the parameters specified here.
This is useful for decrypting application images by sending request
to TIFS-MCU.
For recommendation on the salt, please refer to \ref SECURE_BOOT.

##### Elements of the extension:

salt:
The salt is limited to be 32bytes and is used for key derivation

This OID is ignored as part of HSM Runtime certificates. This OID is ignored
as part of SBL certificates if the device type is HS-FS.
Key derivation is always done using the active user symmetric key.

#### Debug OID (1.3.6.1.4.1.294.1.8)

\code
Debug::= SEQUENCE {
      uid          OCTET STRING     -- Device unique ID
      debugType    INTEGER          -- Debug type
      coreDbgEn    INTEGER          -- Enable core debug mask
      secCoreDbgEn INTEGER          -- Enable secure core debug mask
}
\endcode
The debug object identifier if specified allows the debug ports to be enabled
for a specific device.

##### Elements of the extension:

**UID**: This is the unique identifier associated with the device.
Device specific unique identifiers can be retrieved using the SOC_ID Parser.

The UID field of all 0s is considered to be a wildcard. This is what is supported
by default in the script.

**Debug Type**:
This is the privilege level of debug.

The privilege levels supported by ROM are as follows:

Privilege Level   | Value | Description
------------------|-------|--------------------
DBG_PERM_DISABLE  |	0     | Disable debug ports for all cores.
DBG_SOC_DEFAULT   | 1     | Maintain debug ports for all cores to device type defaults.
DBG_PUBLIC_ENABLE | 2     |	Enable debug ports on R5FSS0-0 core.

coreDbgEn and secCoreDbgEn: These fields are not used and will be ignored.

\note The Debug OID is not applicable for HSM Runtime (TIFS-MCU) image.

In the SBL Signing scripts, `DEBUG_OPTION` is used to exercise the debug level
if the SBL certificate has debug extension. The SBL certificate as mentioned
before, is processed by ROM. This is different from the debug certificate
supported by TIFS-MCU.

This feature is leveraged via compile time flags in MCU+ SDK for SBL images.

By default, in SDK, the debug extension is not engaged for SBL images.
This is decided by the flag `DEBUG_TIFS` defined in `devconfig.mak`.
By default, `DEBUG_TIFS` is set to `yes`.This means that the debug extension
will not be added to SBL and if the debug needs to be exercised via the certificate
to TIFS-MCU via the debug authentication service, it can be safely done.
This is done to ensure that the registers are written once only to exercise the
debug during a POR cycle.
If the user wants to open up debug on an HS-SE device, they can set `DEBUG_TIFS=no`.
This is when the debug extension is added to the certificate for SBL images.

\note ROM does not allow FULL_ENABLE privilege level via SBL certificate. To open up
debug on HSM core, please refer to Debug Authentication services provided via
TIFS-MCU. In this case, please use the default value of `DEBUG_TIFS` i.e. `yes`
when compiling SBL examples.

The default debug privilege used in SDK is `DBG_SOC_DEFAULT`.

To enable debug on public cores on HS-SE device, one can use the following command
to compile SBL examples.

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
\code
make -s -C examples/drivers/boot/sbl_qspi/am263x-cc/r5fss0-0_nortos/ti-arm-clang all DEVICE=am263x DEVICE_TYPE=HS DEBUG_TIFS=no DEBUG_OPTION=DBG_PUBLIC_ENABLE
\endcode
\endcond

\cond SOC_AM273X
\code
make -s -C examples/drivers/boot/sbl_qspi/am273x-cc/r5fss0-0_nortos/ti-arm-clang all DEVICE=am273x DEVICE_TYPE=HS DEBUG_TIFS=no DEBUG_OPTION=DBG_PUBLIC_ENABLE
\endcode
\endcond

\cond SOC_AWR294X
\code
make -s -C examples/drivers/boot/sbl_qspi/awr294x-evm/r5fss0-0_nortos/ti-arm-clang all DEVICE=awr294x DEVICE_TYPE=HS DEBUG_TIFS=no DEBUG_OPTION=DBG_PUBLIC_ENABLE
\endcode
\endcond

\endcond

 - Here,
\cond SOC_AM64X || SOC_AM243X
  - `SBL_RUN_ADDRESS` is `0x70000000`
  - In the case of GP device, `BOOTIMAGE_CERT_KEY` is `rom_degenerateKey.pem`
  - In the case of HS device, `BOOTIMAGE_CERT_KEY` is `custMpk_am64x_am243x.pem`. For more details about this see \ref SECURE_BOOT
\endcond
\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
  - `SBL_RUN_ADDRESS` is `0x70002000`
  - In the case of HS-FS devices, the key value is disregarded. A degenerate RSA public key is used to sign the certificate. The image integrity is checked in ROM, nevertheless.
  - In the case of HS-SE device, more details can be found at \ref SECURE_BOOT
  - Refer to TRM Chapter on Initialization, section 5.6.4.1.1 to get help on RSA key pair generation.
\endcond


These scripts are invoked in makefiles, and the image generation happens
automatically along with the example build. So mostly these scripts need
not be manually run.
If the user build-system is different from TI's makefile system, it needs to
be ensured that the same is followed as part of the post build steps.
The devconfig has ENC_SBL_ENABLED=yes and that is why for HS-SE devices, the SBL
image is encrypted by default.

\cond SOC_AM263X || SOC_AM263PX || SOC_AM273X ||SOC_AWR294X || SOC_AM261X
### Application Signing

For Application images, the script `/source/security/security_common/tools/boot/signing/mcu_appimage_x509_cert_gen.py`.

For HS-SE devices:
\code
cd ${SDK_INSTALL_PATH}/source/security/security_common/tools/boot/signing
$(PYTHON) mcu_appimage_x509_cert_gen.py --bin <path-to-the-binary> --key <signing-key-configured-in-devconfig>  --enc y --enckey <encryption-key-configured-in-devconfig>  --kd-salt  <path-to-key-derivation-salt> --output <output-image-name>
\endcode

Signing of application images is not required for HS-FS device types.

Here, we deep dive into the extension supported with certificate for Application images
as processed by TIFS-MCU.
To get access to the HSM Runtime code and ability to build it for HS-SE device,
please contact your TI represntative to get access to the TIFS-MCU package from
MySecureSW.

#### Boot Information OID (1.3.6.1.4.1.294.1.1)
\code
bootInfo ::=  SEQUENCE {
	cert_type:  INTEGER,		-- identifies the certificate type
	boot_core:	INTEGER,		-- Reserved
	core_opts:	INTEGER,		-- Reserved
	load_addr:	OCTET STRING,	-- Reserved
	image_size:	INTEGER,		-- Image size in bytes
}
\endcode
The Boot Information Object identifier provides information about the image
which is being loaded.

##### Elements of the extension:
**Certificate Type**: The certificate type defines the type of the image which is
being loaded by the Boot-ROM.

These are the supported values:
- 0xA5A50000	Application Image

**Boot core**: This field is reserved and should be assigned with 0.

**Core Options**: This field is reserved and should be assigned with 0.

**Load Address**: This field is reserved and should be assigned with 0.

**Image Size**: This is the size in bytes of the R5 SBL or HSM Runtime Image
to which the certificate has been attached.

#### Image Integrity OID (1.3.6.1.4.1.294.1.2)

\code
imageIntegrity ::= SEQUENCE {
	sha_type:	OID,			-- Identifies the SHA type
	hash:		OCTET STRING	-- The SHA of the boot image
}
\endcode

If the X.509 certificate provides the image integrity boot extension,
the TIFS-MCU will perform the hash calculation on the entire image and will verify
the computed hash with the hash provided in the boot extension.
In the case of a mismatch the boot will fail.

##### Elements of the extension:

**SHA Type**:

The following values are supported:

- 2.16.840.1.101.3.4.2.1	SHA-256 Object Identifier
- 2.16.840.1.101.3.4.2.2	SHA-384 Object Identifier
- 2.16.840.1.101.3.4.2.3	SHA-512 Object Identifier

Amongst these, SDK has been validated against SHA-512 OID.

**Hash**: This is SHA-512 hash which is calculated over the application image

#### Software Revision OID (1.3.6.1.4.1.294.1.3)

\code
softwareRevision ::= SEQUENCE {
   revision:	INTEGER -- Software revision
}
\endcode

The information in the software revision is used to indicate the version of
the image which is being loaded.

##### Elements of the extension:

**revision**:
This is the version number. This will be matched to the EFUSE programmed
version to indicate if the image loading should be done or not.

The following fields in efuse ROM help support the above feature:
- SWRV_APP- This is used to perform the revision checking while loading the application

For more information, about how to fuse in these values, please refer to OTP
Keywriter Documentation available on MySecureSW.

#### Image Encryption OID (1.3.6.1.4.1.294.1.4)

\code
imageEncryption ::= SEQUENCE {
   iv:		OCTET STRING -- The initialization vector
   rs:	    OCTET STRING -- Random string
   iter:	INTEGER      -- Reserved
   salt:	OCTET STRING -- Reserved
}
\endcode

TIFS-MCU only supports AES-CBC mode with 256bit keys. The information in the image encryption object identifier is used to decrypt the image.

##### Elements of the extension:

**IV**:
The initialization vector is used during the AES-CBC decryption procedure. The initialization vector needs to be 16bytes.

**rs**:
This is the random string which is 32bytes long and is appended by the script
at the end of the image. TIFS-MCU will decrypt the image and will perform
a random string comparison to determine if the decryption was successful.

**iter**:
This field is unused and reserved.

**salt**:
This field is unused and reserved.

#### Keyring Index OID (1.3.6.1.4.1.294.1.12)

\code
keyring_index ::= SEQUENCE {
   sign_key_id:		INTEGER -- index of public key hash in public keyring for authentication
   enc_key_id:	    INTEGER -- index of aes key in keyring for decryption
}
\endcode

TIFS-MCU only supports authentication with keyring. The information in enc_key_id is ignored and decryption is done against root of trust
if Image Encryption OID is available in the certificate.

##### Elements of the extension:

**sign_key_id**:
 This is the index used in public keyring for retrieving the public key hash for authentication.

 **enc_key_id**:
 This is the index used in keyring for retrieving the aes key for decryption of application image.
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

\cond SOC_AM263X || SOC_AM263PX || SOC_AM273X || SOC_AM261X
## Keyring Cert Generation Python Script {#KEYRING_CERT_GEN_PYTHON_SCRIPT}

\note Support for encryption with symmetric auxiliary keys will be added in future releases.

\imageStyle{keyring_tooling_view.png,width:50%}
\image html keyring_tooling_view.png "Keyring certificate generation and import"

\code
$python3 keyring_cert_gen.py --root_key ../../source/security/security_common/tools/boot/signing/mcu_custMpk.pem --keys_info keys.json
\endcode

- This script is used to generate X.509 certificate for keyring.
- Make sure that python3 and its dependent modules are installed in the host machine as mentioned in \ref INSTALL_PYTHON3
- keyring_cert_gen.py parses json object containing the following meta-data for keyring cert generation:
    - keyring_sw_rev: This integer value is used for anti-rollback check against the keyring software revision available in device efuses.
    - keyring_ver: Keyring version is of type integer and is the version which is supported against the TIFS-MCU version being used in HSM.
    - num_of_asymm_keys: Number of Asymmetric keys being imported.
    - num_of_symm_keys: Number of Symmetric keys being imported.
    - keyring_asymm: Array of asymmetric keys with respective key rights and hash algorithm to compute and store the hash of the public key.
\code
{
    "keyring_sw_rev" : 1,
    "keyring_ver" : 1,
    "num_of_asymm_keys" : 7,
    "num_of_symm_keys" : 0,
    "keyring_asymm" :  [
                {
                    "key_rights": "000000AA",
                    "pub_key": "aux_keys/public2k.pem",
                    "hash_algo": "SHA256"
                },
                {
                    "key_rights": "000000AA",
                    "pub_key": "aux_keys/public3k.pem",
                    "hash_algo" : "SHA512"
                },
                {
                    "key_rights": "000000AA",
                    "pub_key": "aux_keys/public4k.pem",
                    "hash_algo" : "SHA512"
                },
                {
                    "key_rights": "000000AA",
                    "pub_key": "aux_keys/prime265v1_public.pem",
                    "hash_algo" : "SHA512"
                },
                {
                    "key_rights": "000000AA",
                    "pub_key": "aux_keys/secp384r1_public.pem",
                    "hash_algo" : "SHA512"
                },
                {
                    "key_rights": "000000AA",
                    "pub_key": "aux_keys/secp521r1_public.pem",
                    "hash_algo" : "SHA384"
                },
                {
                    "key_rights": "000000AA",
                    "pub_key": "aux_keys/brainpoolp512r1_public.pem",
                    "hash_algo" : "SHA384"
                }
      ],
    "keyring_symm" :  []
}

\endcode
- Each asymmetric key json object contain 3 fields:
    - key_rights: Integer value in HEX format which signifies the rights associated with the key.
        - debugAuth (0-3b) - flag to indicate key right for debug certificate authentication
        - imageAuth (4-7b) - flag to indicate key right for application image authentication
        - For example: 0x000000AA represents the public can be used for both imageAuth and debugAuth
    - pub_key: Location of the public key
    - hash_algo: Hash algorithm used for hashing public key.
        - SHA256, SHA384 and SHA512 are valid hash algorithms.
- keyring_cert_gen.py expects 2 mandatory arguments:
    - root_key: certificate is signed using customer MPK and expects path to customer active ROT.
    - keys_info: json file with keyring meta data.
- If RSASSA-PSS algorithm for authentication of keyring certificate is required, provide the --rsassa_pss flag and --pss_saltlen **value**. Maximum supported salt length value for RSASSA-PSS is **255**. Please refer to the command below
\code
$python3 keyring_cert_gen.py --root_key ../../source/security/security_common/tools/boot/signing/mcu_custMpk.pem --keys_info keys.json --rsassa_pss --pss_saltlen 64
\endcode
- A dummy json and keyring certificate header file is also availble in ${SDK_INSTALL_PATH}/tools/keyring_cert for reference.
- Below are the logs after execution of script
    \code
keyring version = 1
keyring software revision = 1
Number of asymmetric keys = 7
Number of symmetric keys = 0

[ keyring_asymm ]
asymm_key0=SEQUENCE:comp0
asymm_key1=SEQUENCE:comp1
asymm_key2=SEQUENCE:comp2
asymm_key3=SEQUENCE:comp3
asymm_key4=SEQUENCE:comp4
asymm_key5=SEQUENCE:comp5
asymm_key6=SEQUENCE:comp6

[ comp0 ]
keyId=INTEGER:32
key_rights=FORMAT:HEX,OCT:000000AA
hash_algo=INTEGER:6
public_key=FORMAT:HEX,OCT:582fc2a6cc997d2df6c0299aeef0a7b606d3fcc2261dfade8c2684c6663ab50b

[ comp1 ]
keyId=INTEGER:33
key_rights=FORMAT:HEX,OCT:000000AA
hash_algo=INTEGER:4
public_key=FORMAT:HEX,OCT:6e430c4a09572240f9acd4b4777b2d662fe28c73ef475280d8a4e7c766d13680b14549a84e6e6c7b69a0ba33840dc9f51f110afb156251a6139c26693e6d4ddf

[ comp2 ]
keyId=INTEGER:34
key_rights=FORMAT:HEX,OCT:000000AA
hash_algo=INTEGER:4
public_key=FORMAT:HEX,OCT:67cd4b91e7966af635a53642102abb2c8fdab1a47277c88eb4ec3e1a6ce5155a2325f66b3d392f3f5091e526057cd5e6b6d5085ae46f169c61de77c7dffbb238

[ comp3 ]
keyId=INTEGER:35
key_rights=FORMAT:HEX,OCT:000000AA
hash_algo=INTEGER:4
public_key=FORMAT:HEX,OCT:79bd183d942cbf1542a2bad2a469242392c515c1546fc8a43e86a024097ba4bb4db65ccade7fb6b252deccd255491151df927f998d468153072cf3e4949d6002

[ comp4 ]
keyId=INTEGER:36
key_rights=FORMAT:HEX,OCT:000000AA
hash_algo=INTEGER:4
public_key=FORMAT:HEX,OCT:4a8959978336886acbe3cf294b1b8e1aa68f1cb836d78aa11332a57cb44c435c82fc672901a2a242e5169543502106ac0cda1b04a9769723154d640e832298ba

[ comp5 ]
keyId=INTEGER:37
key_rights=FORMAT:HEX,OCT:000000AA
hash_algo=INTEGER:2
public_key=FORMAT:HEX,OCT:1b759f0c0b04796cc599033bd58e33730ce6b2380f7442fe030beffa3cc844ce0d196f48e5ebeb6d88eea0f7ddc3beb7

[ comp6 ]
keyId=INTEGER:38
key_rights=FORMAT:HEX,OCT:000000AA
hash_algo=INTEGER:2
public_key=FORMAT:HEX,OCT:b502e951a5f5ed4bc99191511b530597d3a2d356d0f83887a54253a7ec46fb2c081c7d39391f846932357a57133f8c11
    \endcode


\endcond

