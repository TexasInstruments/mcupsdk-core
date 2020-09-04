# Security {#SECURITY_PAGE}

[TOC]

# Introduction
\cond SOC_AM263X

## Getting started
The HSM or Hardware security Module is a subsystem that acts as the secure
host by offering security services to the rest of the system.
TI offers HS-FS (High Security - Field Securable) as the primary device for
the customers. The MCU+ SDK supports HS-FS device type for AM263x.
It supports loading the HSM firmware (TIFS-MCU) via SBL (Secondary Boot
Loader), which enables access to the hardware resources for cryptographic
operations for R5FSS.

###  Bootloader_socLoadHsmRtFw
The SBL for AM263x supports for loading firmware on the HSM. The firmware is
provided in an encrypted form at `source/drivers/hsmclient/soc/hsmRtImg.h`.
The function `Bootloader_socLoadHsmRtFw` sends a message to ROM and ROM
loads the HSM firmware. After the HSM firmware is loaded and has done
its init time operations, it sends a message to the SBL called
`HSM_MSG_BOOT_NOTIFY`.

\code

int main()
{
    ...
	Drivers_open();
    Bootloader_profileAddProfilePoint("Drivers_open");

    DebugP_log("\r\n");
    Bootloader_socLoadHsmRtFw(gHsmRtFw, HSMRT_IMG_SIZE_IN_BYTES);
    Bootloader_profileAddProfilePoint("LoadHsmRtFw");

    DebugP_log("Starting QSPI Bootloader ... \r\n");
	...
}

\endcode

### Resources available
By default, the access to the crypto resources are firewalled on HS-FS devices.
The HSM firmware so loaded, bypasses the firewalls and makes the following
crypto modules available for the R5FSS to use.

* AES (public context)
* SHA (public context)
* PKA
* TRNG

This provides ability for the R5F core to be able to do the following
computations:

* AES encryption/decryption
* SHA/HMAC hash calculation
* RSA encryption/decryption, signing/verification
* ECDSA encryption/decryption, signing/verification

SBL should always wait for `HSM_MSG_BOOT_NOTIFY` before using the
crypto accelerator because HSM firmware initializes these firewalls.
Failing to do so, the SBL or application may run into abort exception while
accessing the MMR regions for the crypto accelerator.

### Services

The TIFS-MCU firmware that gets loaded on HSM provides the following services.

* Get Version
	- This service when invoked provides the version of the TIFS-MCU firmware
      loaded on HSM. Please refer to \ref EXAMPLES_HSM_SERVICES.

* Get UID
	- This service when invoked provides the unique identifier (UID) of the device.
       Please refer to \ref EXAMPLES_HSM_SERVICES.

\endcond

### Modules

This page links to sub modules that enable authentication, data integrity and
its confidentiality with the on-chip hardware accelarators.

The cryptographic accelarator on this device is supported via these modules in
the SDK:
\cond SOC_AM64X || SOC_AM243X
- Cryptography Modules
    - \subpage SECURITY_SA2UL_MODULE_PAGE (Ultra lite Security Accelerator)
        - Description of SA2UL architecture and APIs available to use AES, SHA and RNG engine.
    - \subpage SECURITY_PKA_MODULE_PAGE (Public key accelerator)
        - Description of PKA engine and APIs available to use it.
\endcond
\cond SOC_AM263X
- Cryptography Modules
    - \subpage DRIVERS_DTHE_PAGE (Data Transform and Hashing Engine)
        - Description of DTHE architecture and APIs available to use AES and SHA engine.

\note EDMA support for AES and SHA will be added in future releases. Currently,
the drivers only operate in CPU mode of data copy.

\endcond