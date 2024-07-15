# HS FS Migration Guide {#HSFS_MIGRATION_GUIDE}

[TOC]

\cond SOC_AM64X || SOC_AM243X
The MCU+SDK primarily supports **HS-FS** (High Security - Field Securable) as the main device and the support for GP is deprecated. This document aims to list out
the differences between the two device types and aid the users to develop on HS-FS device type.

## System Firmware

One of the major differences between GP and HS-FS is System Firmware (SYSFW).
In both devices, SYSFW is always signed with an asymmetric key (RSA/ECC) and
wrapped in the resulting x509 certificate. In case of GP, the key used for signing
is not verified by ROM, so usually a degenerate key is used. In case of HS-FS device,
SYSFW will be encrypted with TI encryption key (TI MEK) and signed with TI asymmetric key (TI MPK).
The SDK will contain the encrypted and signed binary.

\imageStyle{sysfw_hs_fs.png,width:40%}
\image html sysfw_hs_fs.png "SYSFW in HS-FS device"

## NON-SBL Example Appimage Build

In case of example applications other than SBL (hello_world, dpl_demo etc...)
the same application image file (`*.appimage`) can be booted by SBL on HS-FS / GP device,
nothing special to be done for HS-FS since there is no extra signing needed.

Note: Appimages would eventually need signing by x509Certificate in HS-FS. This
change is yet to be implemented. Once this is done, the appimages would have different
extensions for HS-FS image. `*.appimage` would now become `*.appimage.hs_fs`

## SBL Application Build

In case of SBL applications (sbl_sd, sbl_ospi, sbl_null etc...) since SDK follows
combined boot flow, SYSFW is packaged along with the sbl final image (`*.tiimage`).
Since the SYSFW is different between GP and HS-FS, the sbl final image also will change.
This is taken care through an additional keyword in the file extension, **'hs_fs'**.
The `*.tiimage` would now become `*.hs_fs.tiimage` For example in the case of `sbl_ospi`,
with release build profile, the final image in the case of GP device type would be

 - `sbl_ospi.release.tiimage`

In case of HS-FS device type, it would be

 - `sbl_ospi.release.hs_fs.tiimage`

HS-FS images will be automatically built when you build SBLs, so no extra
build options are required to be passed.

## Application boot using SBL (SBL OSPI, SBL NULL, SBL UART, SBL SD etc.) {#SBL_BOOT_HS_FS}

Since most of the differences in HS-FS is related to image signing, once the SBL
and application image is correctly built, rest of the flow is the same as GP for
application boot. Take care that the right images are used.
For example, to use SBL NULL for your application boot,
**flash the `sbl_null.release.hs_fs.tiimage`** image. Same goes for other SBLs.

\attention Please make sure that you use the SBL images with the `*.hs_fs.tiimage`
and application images `*.appimage.hs_fs` extension while flashing.

## Application boot using CCS + GELs {#CCS_BOOT_HS_FS}

This section refers to the flow where users do not use any SBL, instead uses
CCS + GELs and load_dmsc_hsfs.js to initialize SoC, load the DMSC M3 and manually load
user application to desired core.

- M3 JTAG is locked, so can't connect to M3 and run GELs

Steps to do DMSC initialization from CCS

- Set the board in DEV-BOOT mode (mandatory)
- Power ON the EVM/board
- Launch the CCS target configuration file
- Run the load_dmsc_hs_fs.js script from the scripting console

Since it can't connect to M3, it connects to R5. Now it runs an application (`sciclient_ccs_init`).
The `sciclient_ccs_init` application will load
load M3 with SYSFW and set the boardcfg and initialize the other cores.

The major difference here is the strict adherence of the boot mode to DEV-BOOT mode.
This is necessary to be able to connect to the R5 and load the sciclient_ccs_init
application. The DEV BOOT mode looks like below:

\imageStyle{boot_pins_devboot_mode.png,width:40%}
\image html boot_pins_devboot_mode.png "DEV BOOT MODE"

To reiterate, to do application boot using CCS and GELs in HS-FS device

- Set the board in DEV-BOOT mode (mandatory)
- Power ON the EVM/board
- Launch the CCS target configuration file
- Run the load_dmsc_hs_fs.js script from the scripting console
- Wait for the SYSFW version to be printed on CCS console
- Manually reset the R5FSS0_0 core and proceed towards application load

## Difference of SYSFW loading in SBL mode and CCS mode in HS-FS

In the SBL based flow, the SBL image will contain the SYSFW in the appropriate
format and the ROM will load the SBL and SYSFW. In this scenario, the above mentioned
sysfw-hs-fs-enc.bin and sysfw-hs-fs-enc-cert.bin binaries would be used in the image
creation. In case of CCS based flow, the loading is done in a different way using
an R5 application. Here we have to use a binary concatenation of the certificate
and the encrypted image and generate a hex header out of it. This hex header is
then included by the R5 application and then loaded. This is already done and
the header file is kept at the same location as other SYSFW binaries - `sysfw_hs_fs_signed.h`

## Firewall Configurations

Since HS-FS devices has some the secure services, the firewall configurations are
somewhat different compared to the fully open and non-secure GP devices. For example
some of the SoC memory regions are firewalled in HS-FS devices. This includes the
MSRAM, which cores or DMA engines might need to access. To tackle this, all the SBLs
in the SDK opens the MSRAM firewalls right after SYSFW boot notification comes up.
If you write your own bootloader, you should also do this. The API
`Bootloader_socOpenFirewalls` can be used for this. For example usage, please check
the source of any SBL.

## Summary of differences in GP vs HS-FS

| Entity                     | GP                                           | HS-FS
-----------------------------|----------------------------------------------|--------------------------------------------------------------------------------------------
| SYSFW image                | Plain (`sysfw.bin`)                          | Signed and encrypted with TI Key (`sysfw-hs-fs-enc.bin`, `sysfw-hs-fs-enc-cert.bin`)
| Application image name     | `*.appimage`                                 | `*.appimage.hs_fs` (eventually)
| SBL image name             | `*.tiimage`                                  | `*.hs_fs.tiimage`
| CCS + GEL flow bootmode    | NO-BOOT recommended, any bootmode functional | DEV-BOOT mode mandatory
| CCS + GEL flow js script   | `load_dmsc.js`                               | `load_dmsc_hs_fs.js`
| Firewall Configurations    | Firewalls Open                               | Some regions are firewalled

## Signing of Images in GP vs HS-FS

SBL images and non-SBL example application images use different signing scripts.
This is due to the difference in the x509 certificate format expected by ROM and
SYSFW. These differences and scripting is already part of SDK makefiles, but in
case if the user has their own build system and wants to integrate these steps
into it, please go through this section. The signing scripts used are python3 based.

### SYSFW Signing

SYSFW is not signed with SDK signing scripts. Pre-encrypted SYSFW binaries with
x509 certificate binary generated as a result of signing with the TI keys is
part of the SDK.

### SBL Signing

To create the x509 certificate header-ed SBL image, the script used is `rom_image_gen.py`.
This is located in `${SDK_ROOT}/tools/boot/signing`. The usage of the script will be printed if you run it with `-h` option like so:

\code
$ python3 rom_image_gen.py -h
usage: rom_image_gen.py [-h] [--swrv SWRV] --sbl-bin SBL_BIN [--sbl-enc] [--enc-key ENC_KEY] --sysfw-bin
                        SYSFW_BIN [--sysfw-inner-cert SYSFW_INNER_CERT] --boardcfg-blob BOARDCFG_BLOB
                        --sbl-loadaddr SBL_LOADADDR --sysfw-loadaddr SYSFW_LOADADDR --bcfg-loadaddr
                        BCFG_LOADADDR --key KEY --rom-image ROM_IMAGE [--debug DEBUG]

Creates a ROM-boot-able combined image when the SBL, SYSFW and BoardCfg data are provided

optional arguments:
  -h, --help            show this help message and exit
  --swrv SWRV           Software revision number
  --sbl-bin SBL_BIN     Path to the SBL binary
  --sbl-enc             Encrypt SBL or not
  --enc-key ENC_KEY     Path to the SBL Encryption Key
  --sysfw-bin SYSFW_BIN
                        Path to the sysfw binary
  --sysfw-inner-cert SYSFW_INNER_CERT
                        Path to the sysfw inner certificate
  --boardcfg-blob BOARDCFG_BLOB
                        Path to the boardcfg blob
  --sbl-loadaddr SBL_LOADADDR
                        Load address at which SBL needs to be loaded
  --sysfw-loadaddr SYSFW_LOADADDR
                        Load address at which SYSFW needs to be loaded
  --bcfg-loadaddr BCFG_LOADADDR
                        Load address at which BOARDCFG needs to be loaded
  --key KEY             Path to the signing key to be used while creating the certificate
  --rom-image ROM_IMAGE
                        Output file combined ROM image of SBL+SYSFW+Boardcfg
  --debug DEBUG         Debug options for the image
\endcode

The script is usually run after the sbl binary is created using an objcopy utility.
That is a `*.bin` file. In GP, we sign the binary using a degenerate key. We also
pass the GP SYSFW binary and the boardcfg binary to the signing script. Here is
an example of how it's done in GP for say `sbl_ospi`.

\code
$ python3 rom_image_gen.py --swrv 1 --sbl-bin /path/to/sbl_ospi.release.bin --sysfw-bin /path/to/sysfw.bin --boardcfg-blob /path/to/boardcfg_blob.bin --sbl-loadaddr 0x70000000 --sysfw-loadaddr 0x44000 --bcfg-loadaddr 0x7B000 --key /path/to/rom_degenerateKey.pem --rom-image /path/to/sbl_ospi.release.tiimage
\endcode

In the options mentioned here, the values used for SYSFW load address (`--sysfw-loadaddr`)
and BoardCfg load address (`--bcfg-loadaddr`) are fixed for an SoC, so no need to change.
For SBL load address, we generally keep the top part of the MSRAM reserved for loading
the SBL in the SDK. So we give the start of MSRAM, `0x70000000`. If your SBL
needs to be loaded elsewhere, this value can be changed. Notice how just the
SYSFW binary is passed in case of GP. This is an **unsigned, unencrypted binary**.

In case of HS-FS device, we use the same signing script, but there is a small difference in the usage.

\code
$ python3 rom_image_gen.py --swrv 1 --sbl-bin /path/to/sbl_ospi.release.bin --sysfw-bin /path/to/sysfw-hs-fs-enc.bin --sysfw-inner-cert /path/to/sysfw-hs-fs-enc-cert.bin--boardcfg-blob /path/to/boardcfg_blob.bin --sbl-loadaddr 0x70000000 --sysfw-loadaddr 0x44000 --bcfg-loadaddr 0x7B000 --key /path/to/rom_degenerateKey.pem --rom-image /path/to/sbl_ospi.release.hs_fs.tiimage
\endcode

You can see that we are involving the x509 certificate for the HS-FS SYSFW binary
in the image creation. This is the major difference in the SBL image creation of HS-FS devices.
You would observe that the other parameters are pretty much the same between both cases.

## Appimage Signing

Signing for the application image is new in HS-FS compared to GP. For the signing
of application images we use a different script. This is because authentication
of application images are done by SYSFW and it expects a different x509 certificate format.
For signing application images, the script used in `appimage_x509_cert_gen.py`
located at `${SDK_ROOT}/source/security/security_common/tools/boot/signing/`.
The usage of the script will be printed if you run it with `-h` option like so:

\code
python3 appimage_x509_cert_gen.py -h
usage: appimage_x509_cert_gen.py [-h] [--bin BIN] [--key KEY] [--enckey ENCKEY] [--cert CERT] [--output OUTPUT] [--core CORE] [--enc ENC] [--loadaddr LOADADDR] [--authtype AUTHTYPE]

Generates a x509 certificate for an application binary to boot it in HS device

optional arguments:
  -h, --help           show this help message and exit
  --bin BIN            Bin file that needs to be signed
  --key KEY            File with signing key inside it
  --enckey ENCKEY      File with encryption key inside it
  --cert CERT          Certificate file name (optional, will use a default name otherwise)
  --output OUTPUT      Output file name (concatenated cert+bin)
  --core CORE          Core on which the binary is meant to be loaded. Optional
  --enc ENC            If the binary need to be encrypted or not [y/n]
  --loadaddr LOADADDR  Target load address of the binary in hex. Default to 0x70000000
  --authtype AUTHTYPE  Authentication type. [0/1/2]. 0 - Move to destination address specified after authentication, 1 - In place authentication, 2 - Move to the certificate start after authentication. Default is 1
\endcode

Usage of the signing script in HS-FS is fairly simple compared to the SBL signing script.
Here is a typical usage for `hello_world` example:

\code
$ python3 appimage_x509_cert_gen.py --bin /path/to/hello_world.release.appimage --authtype 1 --key /path/to/rom_degenerateKey.pem --output /path/to/hello_world.release.appimage.hs_fs
\endcode

Note that the key used for signing is rom degenerate.

\endcond