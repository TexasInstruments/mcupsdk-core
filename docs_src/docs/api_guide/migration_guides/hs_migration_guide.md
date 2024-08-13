# HS Migration Guide {#HS_MIGRATION_GUIDE}

[TOC]

\cond SOC_AM65X
The MCU+SDK primarily supported **GP** (General Purpose) as the main device type.
MCU+SDK will now support **HS** (High Security) in the SDK
as the main device type with time-limited support for GP. This document aims to list out
the differences between the two device types and aid the users to develop on HS device type.

## System Firmware

One of the major differences between GP and HS is System Firmware (SYSFW).
In both devices, SYSFW is always signed with an asymmetric key (RSA/ECC) and
wrapped in the resulting x509 certificate. In case of GP, the key used for signing
is not verified by ROM, so usually a degenerate key is used. In case of HS device,
SYSFW will signed with TI asymmetric key (TI MPK).
The SDK will contain the encrypted and signed binary.

## NON-SBL Example Appimage Build

In case of example applications other than SBL (hello_world, dpl_demo etc...)
the application image file (`*.appimage`) booted by SBL on GP device, (`*.appimage.hs`) booted by SBL on HS device.

Note: Appimages need signing by x509Certificate in HS.

## SBL Application Build

\attention Please make sure to clean the SBL image before build the HS SBL image.

In case of SBL applications (sbl_sd, sbl_ospi, sbl_null etc...) since SDK follows
un-combined boot flow, SYSFW is not packaged along with the sbl final image (`*.tiimage`).
Since the SYSFW is different between GP and HS, the sbl final image also will change.
This is taken care through an additional keyword in the file extension, **'hs'**.
The `*.tiimage` would now become `*.hs.tiimage` For example in the case of `sbl_ospi`,
with release build profile, the final image in the case of GP device type would be

 - `sbl_ospi.release.tiimage` (Same as before)

In case of HS device type, it would be

 - `sbl_ospi.release.hs.tiimage`

## Application boot using SBL (SBL OSPI, SBL NULL, SBL UART, SBL SD etc.) {#SBL_BOOT_HS}

Since most of the differences in HS is related to image signing, once the SBL
and application image is correctly built, rest of the flow is the same as GP for
application boot. Take care that the right images are used.
For example, to use SBL NULL for your application boot,
**flash the `sbl_null.release.hs.tiimage`** image. Same goes for other SBLs.

\attention Please make sure that you use the SBL images with the `*.hs.tiimage`
and application images `*.appimage.hs` extension while flashing.

## Firewall Configurations

Since HS devices has some the secure services, the firewall configurations are
somewhat different compared to the fully open and non-secure GP devices. For example
some of the SoC memory regions are firewalled in HS devices. This includes the
MSRAM, which cores or DMA engines might need to access. To tackle this, all the SBLs
in the SDK opens the MSRAM firewalls right after SYSFW boot notification comes up.
If you write your own bootloader, you should also do this. The API
`Bootloader_socOpenFirewalls` can be used for this. For example usage, please check
the source of any SBL.

## Summary of differences in GP vs HS

| Entity                     | GP                                           | HS
-----------------------------|----------------------------------------------|--------------------------------------------------------------------------------------------
| SYSFW image                | Plain (`sysfw.bin`)                          | Signed and encrypted with TI Key (`sysfw_sr2-hs-enc.bin`)
| Application image name     | `*.appimage`                                 | `*.appimage.hs` (eventually)
| SBL image name             | `*.tiimage`                                  | `*.hs.tiimage`
| Firewall Configurations    | Firewalls Open                               | Some regions are firewalled

## Signing of Images in GP vs HS

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

To create the x509 certificate header-ed SBL image, the script used is `x509CertificateGen.ps1`.
This is located in `${SDK_ROOT}/tools/boot/signing`.

In GP, we sign the binary using a degenerate key. Here is
an example of how it's done in GP for say `sbl_uart`.

\code
$ python3 x509CertificateGen.ps1 -b sbl_uart.release.bin -o sbl_uart.release.tiimage -c R5 -l 0x41C00100 -k rom_degenerateKey.pem -d DEBUG -j DBG_FULL_ENABLE -m SPLIT_MODE
\endcode

In case of HS device, we use the same signing script, but there is a small difference in the usage.

\code
$ python3 x509CertificateGen.ps1 -b sbl_uart.release.bin -o sbl_uart.release.hs.tiimage -c R5 -l 0x41C00100 -k k3_dev_mpk.pem -d DEBUG -j DBG_FULL_ENABLE -m SPLIT_MODE
\endcode

You can see that we are involving the x509 certificate for the HS SYSFW binary
in the image creation. This is the major difference in the SBL image creation of HS devices.
You would observe that the other parameters are pretty much the same between both cases.

## Appimage Signing

Signing for the application image is new in HS compared to GP. For the signing
of application images we use a different script. This is because authentication
of application images are done by SYSFW and it expects a different x509 certificate format.
For signing application images, the script used in `appimage_x509_cert_gen.py`
located at `${SDK_ROOT}/tools/boot/signing/`.

Usage of the signing script in HS is fairly simple compared to the SBL signing script.
Here is a typical usage for `i2c_led_blink` example:

\code
$ python3 appimage_x509_cert_gen.py --bin i2c_led_blink.release.appimage --authtype 1 --key k3_dev_mpk.pem --output i2c_led_blink.release.appimage.hs
\endcode

\endcond