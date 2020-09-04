# Enabling Secure Boot {#SECURE_BOOT}

[TOC]

## Secure Devices and Secure Boot: An Introduction

To put it simply, **secure boot** refers to booting application images in a secure way. Secure boot is a feature/service available in **secure devices**. Out of the device types **GP** and **HS**, HS device type can do secure boot. In secure devices there are two subtypes:

1. Field Securable (FS)
2. Security Enforced (SE)

HS device type have FS and SE subtypes. Out of the two subtypes, FS and SE, this guide talks about secure boot in an **HS-SE** device. If the secure device received is an **HS-FS** subtype, it needs to be converted to an HS-SE variant with the customer keys burnt into the device eFUSEs. This is done using a special piece of software called an OTP keywriter. Keywriter documentation is out of scope for this guide.

## Secure Boot Process

Secure boot process, like the normal boot, consists of two stages - ROM loading and SBL loading. ROM loading is when the boot ROM loads the HSM runtime binary onto the HSM core, and the signed SBL binary into the primary boot core, which in most cases is an ARM Cortex R5F. SBL loading is when the SBL reads the signed application image from a boot media, authenticates it, decrypts it, and boots it. Here we describe how the secure process takes place in an HS-SE device.

\note In AM243x/AM64x devices, the HSM runtime binary mentioned is the System Firmware (SYSFW) and HSM core is the DMSC Cortex M3 core. Hereafter for generality sake we'll use the terms 'HSMRt' and 'HSM core' but understand that for AM64x/AM243x this means the SYSFW and Cortex M3 core.

## Secure Boot Support in SDK

### Device configuration file

To make the secure/non-secure differences seamless for the user, a configuration file is provided at `${SDK_INSTALL_PATH}/devconfig/devconfig.mak`.
In this configuration file, you can set certain options like the device type, keys to be used for signing and encryption etc. By default they will point to the dummy customer MPKs and MEKs but if you're using a production device with your own keys burned into the eFUSEs, please change the paths here to point to the right key files. Configuration of this file is currently manual, this will be made configurable by a GUI in an upcoming release.

The devconfig.mak file looks something like this:

\code
# Device type (HS/GP)
DEVICE_TYPE?=GP

# Path to the signing tools, keys etc
SIGNING_TOOL_PATH=$(MCU_PLUS_SDK_PATH)/tools/boot/signing

# Path to the keys
ROM_DEGENERATE_KEY:=$(SIGNING_TOOL_PATH)/rom_degenerateKey.pem
CUST_MPK=$(SIGNING_TOOL_PATH)/custMpk_am64x_am243x.pem
CUST_MEK=$(SIGNING_TOOL_PATH)/custMek_am64x_am243x.txt

# Encryption option (yes/no)
ENC_ENABLED?=no

# Generic macros to be used depending on the device type
APP_SIGNING_KEY=
APP_ENCRYPTION_KEY=

ifeq ($(DEVICE_TYPE),HS)
	APP_SIGNING_KEY=$(CUST_MPK)
	APP_ENCRYPTION_KEY=$(CUST_MEK)
else
	APP_SIGNING_KEY=$(ROM_DEGENERATE_KEY)
endif
\endcode

This file will be included in all example makefiles

### Signing tool

For signing the binaries, two different scripts are used:

1. ROM signing script - This is a python script used for signing the SBL. The x509 certificate template used in this script is expected by the ROM.
2. Application signing script - This is a python script used for signing the application image. The x509 certificate template used in this script is expected by the HSMRt.

 For more details on the usage of the script, see \ref TOOLS_BOOT_SIGNING

### Signing steps in an HS-SE device

#### Signing the SBL

\cond SOC_AM64X || SOC_AM243X
In AM64x/AM243x devices, a combined boot method is employed, by virtue of which the SBL, SYSFW and the SYSFW-BoardConfig are combined and signed with the same certificate. This is built into the make system of the SBL applications in the SDK - SBL_UART, SBL_OSPI, SBL_SD, SBL_NULL, etc. So whenever an SBL application is built, the loadable `*.tiimage` will be a concatenation of the x509 certificate, SBL binary, SYSFW binary and the boardcfg binary blob. In case of HS devices, the SYSFW inner certificate will also be concatenated.
\endcond

The SBL is signed with a dummy customer MPK in the SDK. This is supposed to be used only with the devices with the same dummy customer MPK burnt into the eFUSEes. If the SDK is supposed to be used with a production/development device with actual customer MPKs burnt into the device, please replace the file at ${SDK_INSTALL_PATH}/tools/boot/signing/custMpk_${SOC}.pem. This is true for also the encryption key used, which can also be found at ${SDK_INSTALL_PATH}/tools/boot/signing/custMek_${SOC}.txt. Whenever any SBL is built, it will be signed with dummy customer MPK, and the signed image generated will have an extension of `*.hs.tiimage`. There is no extra step required other than making sure that the MPK used is indeed the one burnt into the eFUSEs.

\note We have enabled full debug while signing the ROM image. This is intentional as this is helpful for debug. Once moved from development to production please remove this option from the makefile. For more details see \ref TOOLS_BOOT_SIGNING

\cond SOC_AM64X | SOC_AM243X
#### Signing the HSM Runtime binary (SYSFW)
As mentioned above, since we follow a combined boot method, SYSFW and SBL is signed with the same certificate using the same key. In case of a GP device this will be a degenerate key for easy parsing from ROM. In the case of an HS device, SYSFW will be already signed with TI MPK (and encrypted). This is then countersigned again with dummy customer MPK during the combined image generation process.
\endcond

#### Signing the application image

Depending on the options given in the device configuration file (`devconfig.mak` mentioned above), appimage is generated for HS devices. If encryption is enabled in the configuration file, the binary will be first encrypted with the key specified and then the certificate will be generated using the customer MPK specified. If the device type is set as HS in the configuration file, nothing extra needs to be done for the appimage generation. The final `*.appimage.hs` file generated would be signed with customer MPK (and encrypted with customer MEK if that option is selected).
\cond SOC_AM64X | SOC_AM243X
To dig into the details of the process, one can refer to https://software-dl.ti.com/tisci/esd/latest/6_topic_user_guides/secure_boot_signing.html
\endcond

The SBL doesn't have innate abilities to do the image integrity check, or verify the SHA512 of the application image. It relies on HSMRt (SYSFW in AM243x/AM64x) for this. The image is stored in a readable memory and a pointer to the start of the image is passed to the HSMRt with other details like load address, type of authentication etc.

\cond SOC_AM64X | SOC_AM243X
For more information regarding the authentication request, please refer to http://downloads.ti.com/tisci/esd/latest/2_tisci_msgs/security/PROC_BOOT.html#proc-boot-authenticate-image-and-configure-processor
\endcond


## Limitations in Secure Boot

- **XIP boot** : Secure boot is yet to be supported for XIP applications. This is due to the fact that the XIP sections are loaded before the SBL parses the other sections. Secure boot of XIP applications will be made available in an upcoming release.

- **Encryption of application image not possible in SBL OSPI** : In other bootloaders like UART and SD, application image can be encrypted using the `ENC_ENABLED` option in the devconfig.mak. But this is not possible when you load the image using SBL OSPI. This is due to the fact that HSM does an in-place authentication and decryption of the image and we load the image directly from the FLASH memory in case of SBL OSPI. FLASH memory, as you would know is most often not directly writable. Due to this limitation not being taken care in the HSM, we can do decryption of images only in the case where the image resides in a volatile RAM-like memory. That is MSMC or DDR.
