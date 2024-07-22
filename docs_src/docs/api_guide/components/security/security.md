# Security {#SECURITY_PAGE}

[TOC]

# Introduction
\cond SOC_AM263X || SOC_AM263PX || SOC_AM273X || SOC_AM261X

## Getting started
The HSM or Hardware security Module is a subsystem that acts as the secure
host by offering security services to the rest of the system.
TI offers HS-FS (High Security - Field Securable) as the primary device for
the customers. The MCU+ SDK supports HS-FS device type for AM263x/AM263Px.
It supports loading the HSM firmware (TIFS-MCU) via SBL (Secondary Boot
Loader), which enables access to the hardware resources for cryptographic
operations for R5FSS.

###  Bootloader_socLoadHsmRtFw
The SBL for AM263x/AM263Px/AM273x supports for loading firmware on the HSM. The firmware is
provided in an encrypted form at `source/security/security_common/drivers/hsmclient/soc/am263x/hsmRtImg.h`.
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

#### HSMRt Firewall Configurations {#SECURITY_HSFS_BOOTTIME_FIREWALL}

HSMRt does the following firewall configurations. These configurations gives necessary access of various memory regions to host cores.
\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X

\note
Following configurations are done considering **R5FSS0_0** as *secure host 0*
and **R5FSS0_1** as *secure host 1*.

\endcond
\cond SOC_AM273X

\note
Following configurations are done considering **R5FSS0_0** as *secure host 0*
and **C66x** as *secure host 1*.

\note
A secure host refers specifically to a host that is configured to communicate with an HSM (Hardware Security Module), rather than simply implying a host that is inherently secure.
\endcond


\cond SOC_AM263X

| **Firewall/MPU**          | **Programmable Region Num.** | **Start Address** | **End Address** | **AID/privID permissions**                          | **SR** | **SW** | **SX** | **UR** | **UW** | **UX** | **NS** | **Debug** | **Comments**                                |
|---------------------------|------------------------------|-------------------|-----------------|-----------------------------------------------------|--------|--------|--------|--------|--------|--------|--------|-----------|---------------------------------------------|
| FW HSM\_SLV               | 0                            | 0x44000400        | 0x44000800      | HSM, R5FSS0\_0,<br>R5FSS0\_1                        | 1      | 1      | 0      | 1      | 1      | 0      | 1      | 1         | HSM MBOX region<br>R5->HSM queues           |
|                           | 1                            | 0x44000000        | 0x44000400      | HSM, R5FSS0\_0,<br>R5FSS0\_1                        | 1      | 0      | 0      | 1      | 0      | 0      | 1      | 1         | HSM MBOX region<br>HSM->R5 queues           |
|                           | 2                            | 0x40020000        | 0x4011FFFF      | R5FSS0\_0,<br>R5FSS0\_1,<br>R5FSS1\_0,<br>R5FSS1\_1 | 1      | 0      | 0      | 1      | 0      | 0      | 1      | 1         | MPU Region Space                            |
|                           | 3                            | 0x40140000        | 0x4023FFFF      | R5FSS0\_0,<br>R5FSS0\_1,<br>R5FSS1\_0,<br>R5FSS1\_1 | 1      | 0      | 0      | 1      | 0      | 0      | 1      | 1         | MPU Region Space                            |
| FW DTHE\_SLV              | 0                            | 0xCE007000        | 0xCE0073FF      | All AIDs can access                                 | 1      | 1      | 1      | 1      | 1      | 1      | 1      | 1         | HSM AES (public context)                    |
|                           | 1                            | 0xCE005000        | 0xCE0053FF      | All AIDs can acces                                  | 1      | 1      | 1      | 1      | 1      | 1      | 1      | 1         | HSM SHA (public context)                    |
|                           | 2                            | 0xCE000000        | 0xCEFFFFFF      | HSM                                                 | 1      | 1      | 1      | 1      | 1      | 1      | 1      | 1         | HSM DTHE and Crpyto                         |
| FW SCRM2SCRP0             | 0                            | 0x53600000        | 0x53600400      | HSM                                                 | 1      | 1      | 1      | 1      | 1      | 1      | 1      | 1         | Secure Asset and is protected during runtime |
|                           | 1                            | 0x50000000        | 0x53600000      | All AIDs can access                                 | 1      | 1      | 1      | 1      | 1      | 1      | 1      | 1         | To access peripheral region master side MPU |
| FW SCRM2SCRP1             | 0                            | 0x53600000        | 0x53600400      | HSM                                                 | 1      | 1      | 1      | 1      | 1      | 1      | 1      | 1         | Secure Asset and is protected during runtime |
|                           | 1                            | 0x50000000        | 0x53600000      | All AIDs can access                                 | 1      | 1      | 1      | 1      | 1      | 1      | 1      | 1         | To access peripheral region master side MPU |
| FW R5SS0\_CORE0\_AHB\_MST | 0                            | 0x53600000        | 0x53600400      | HSM                                                 | 1      | 1      | 1      | 1      | 1      | 1      | 1      | 1         | Secure Asset and is protected during runtime |
|                           | 1                            | 0x50000000        | 0x53600000      | All AIDs can access                                 | 1      | 1      | 1      | 1      | 1      | 1      | 1      | 1         | To access peripheral region master side MPU |
| FW R5SS0\_CORE1\_AHB\_MST | 0                            | 0x53600000        | 0x53600400      | HSM                                                 | 1      | 1      | 1      | 1      | 1      | 1      | 1      | 1         | Secure Asset and is protected during runtime |
|                           | 1                            | 0x50000000        | 0x53600000      | All AIDs can access                                 | 1      | 1      | 1      | 1      | 1      | 1      | 1      | 1         | To access peripheral region master side MPU |
| FW R5SS1\_CORE0\_AHB\_MST | 0                            | 0x53600000        | 0x53600400      | HSM                                                 | 1      | 1      | 1      | 1      | 1      | 1      | 1      | 1         | Secure Asset and is protected during runtime |
|                           | 1                            | 0x50000000        | 0x53600000      | All AIDs can access                                 | 1      | 1      | 1      | 1      | 1      | 1      | 1      | 1         | To access peripheral region master side MPU |
| FW R5SS1\_CORE1\_AHB\_MST | 0                            | 0x53600000        | 0x53600400      | HSM                                                 | 1      | 1      | 1      | 1      | 1      | 1      | 1      | 1         | Secure Asset and is protected during runtime |
|                           | 1                            | 0x50000000        | 0x53600000      | All AIDs can access                                 | 1      | 1      | 1      | 1      | 1      | 1      | 1      | 1         | To access peripheral region master side MPU |


\endcond

\cond SOC_AM273X

| **Firewall/MPU**          | **Programmable Region Num.** | **Start Address** | **End Address** | **AID/privID permissions**                          | **SR** | **SW** | **SX** | **UR** | **UW** | **UX** | **NS** | **Debug** | **Comments**                                |
|---------------------------|------------------------------|-------------------|-----------------|-----------------------------------------------------|--------|--------|--------|--------|--------|--------|--------|-----------|---------------------------------------------|
| FW HSM\_SLV               | 0                            | 0x44000400        | 0x440007FF      | R5FSS0\_0,<br>R5FSS0\_1, DSS                        | 1      | 1      | 0      | 1      | 1      | 0      | 1      | 1         | HSM MBOX region<br>R5/C66->HSM queues       |
|                           | 1                            | 0x44000000        | 0x440003FF      | R5FSS0\_0,<br>R5FSS0\_1, DSS                        | 1      | 0      | 0      | 1      | 0      | 0      | 1      | 1         | HSM MBOX region<br>HSM->R5/C66 queues       |
|                           | 2                            | 0x40020000        | 0x4005FFFF      | R5FSS0\_0,<br>R5FSS0\_1, DSS                        | 1      | 0      | 0      | 1      | 0      | 0      | 1      | 0         | MPU Region Space                            |
|                           | 3                            | 0x40080000        | 0x4021FFFF      | R5FSS0\_0,<br>R5FSS0\_1, DSS                        | 1      | 0      | 0      | 1      | 0      | 0      | 1      | 0         | MPU Region Space                            |
| FW DTHE\_SLV              | 0                            | 0xCE007000        | 0xCE007FFF      | All AIDs can access                                 | 1      | 1      | 1      | 1      | 1      | 1      | 1      | 1         | HSM AES (public context)                    |
|                           | 1                            | 0xCE005000        | 0xCE005FFF      | All AIDs can access                                 | 1      | 1      | 1      | 1      | 1      | 1      | 1      | 1         | HSM SHA (public context)                    |
|                           | 2                            | 0xCE000800        | 0xCE000FFF      | All AIDs can access                                 | 1      | 1      | 1      | 1      | 1      | 1      | 1      | 1         | HSM DTHE (public context)                   |
| FW MSS\_PCRA              | 0                            | 0x2F7A800         | 0x2F7ABFF       | HSM                                                 | 1      | 1      | 1      | 1      | 1      | 1      | 1      | 1         | Secure Asset and is protected during runtime|
|                           | 1                            | 0x2000000         | 0x2F7A7FF       | All AIDs can access                                 | 1      | 1      | 1      | 1      | 1      | 1      | 1      | 1         | To access peripheral region master side MPU |
|                           | 2                            | 0x2F7AC00         | 0x2FFFFFF       | All AIDs can access                                 | 1      | 1      | 1      | 1      | 1      | 1      | 1      | 1         | To access peripheral region master side MPU |

\endcond


\cond SOC_AM263PX || SOC_AM261X

| **Firewall/MPU**          | **Programmable Region Num.** | **Start Address** | **End Address** | **AID/privID permissions**                          | **SR** | **SW** | **SX** | **UR** | **UW** | **UX** | **NS** | **Debug** | **Comments**                                |
|---------------------------|------------------------------|-------------------|-----------------|-----------------------------------------------------|--------|--------|--------|--------|--------|--------|--------|-----------|---------------------------------------------|
| FW HSM\_SLV               | 0                            | 0x44000400        | 0x44000800      | HSM, R5FSS0\_0,<br>R5FSS0\_1                        | 1      | 1      | 0      | 1      | 1      | 0      | 1      | 1         | HSM MBOX region<br>R5->HSM queues           |
|                           | 1                            | 0x44000000        | 0x44000400      | R5FSS0\_0,<br>R5FSS0\_1                             | 1      | 0      | 0      | 1      | 0      | 0      | 0      | 0         | HSM MBOX region<br>HSM->R5 queues           |
|                           | 2                            | 0x40020000        | 0x4011FFFF      | R5FSS0\_0,<br>R5FSS0\_1,<br>R5FSS1\_0,<br>R5FSS1\_1 | 1      | 0      | 0      | 1      | 0      | 0      | 0      | 0         | MPU Region Space                            |
|                           | 3                            | 0x40140000        | 0x4023FFFF      | R5FSS0\_0,<br>R5FSS0\_1,<br>R5FSS1\_0,<br>R5FSS1\_1 | 1      | 0      | 0      | 1      | 0      | 0      | 0      | 0         | MPU Region Space                            |
|                           | 4                            | 0x40260000        | 0x402E03FF      | R5FSS0\_0,<br>R5FSS0\_1,<br>R5FSS1\_0,<br>R5FSS1\_1 | 1      | 0      | 0      | 1      | 0      | 0      | 0      | 0         | MPU Region Space                            |
| FW DTHE\_SLV              | 0                            | 0xCE007000        | 0xCE0073FF      | All AIDs can access                                 | 1      | 1      | 1      | 1      | 1      | 1      | 1      | 1         | HSM AES (public context)                    |
|                           | 1                            | 0xCE005000        | 0xCE0053FF      | All AIDs can acces                                  | 1      | 1      | 1      | 1      | 1      | 1      | 1      | 1         | HSM SHA (public context)                    |
|                           | 2                            | 0xCE000000        | 0xCEFFFFFF      | HSM                                                 | 1      | 1      | 1      | 1      | 1      | 1      | 1      | 1         | HSM DTHE and Crpyto                         |
|                           | 3                            | 0xCE000800        | 0xCE000FFF      | All AIDs can acces                                  | 1      | 1      | 1      | 1      | 1      | 1      | 1      | 1         | HSM DTHE and Crpyto                         |
| FW SCRM2SCRP0             | 0                            | 0x53600000        | 0x53600400      | HSM                                                 | 1      | 1      | 1      | 1      | 1      | 1      | 1      | 1         | Secure Asset and is protected during runtime |
|                           | 1                            | 0x50000000        | 0x53600000      | All AIDs can access                                 | 1      | 1      | 1      | 1      | 1      | 1      | 1      | 1         | To access peripheral region master side MPU |
| FW SCRM2SCRP1             | 0                            | 0x53600000        | 0x53600400      | HSM                                                 | 1      | 1      | 1      | 1      | 1      | 1      | 1      | 1         | Secure Asset and is protected during runtime |
|                           | 1                            | 0x50000000        | 0x53600000      | All AIDs can access                                 | 1      | 1      | 1      | 1      | 1      | 1      | 1      | 1         | To access peripheral region master side MPU |
| FW R5SS0\_CORE0\_AHB\_MST | 0                            | 0x53600000        | 0x53600400      | HSM                                                 | 1      | 1      | 1      | 1      | 1      | 1      | 1      | 1         | Secure Asset and is protected during runtime |
|                           | 1                            | 0x50000000        | 0x53600000      | All AIDs can access                                 | 1      | 1      | 1      | 1      | 1      | 1      | 1      | 1         | To access peripheral region master side MPU |
| FW R5SS0\_CORE1\_AHB\_MST | 0                            | 0x53600000        | 0x53600400      | HSM                                                 | 1      | 1      | 1      | 1      | 1      | 1      | 1      | 1         | Secure Asset and is protected during runtime |
|                           | 1                            | 0x50000000        | 0x53600000      | All AIDs can access                                 | 1      | 1      | 1      | 1      | 1      | 1      | 1      | 1         | To access peripheral region master side MPU |
| FW R5SS1\_CORE0\_AHB\_MST | 0                            | 0x53600000        | 0x53600400      | HSM                                                 | 1      | 1      | 1      | 1      | 1      | 1      | 1      | 1         | Secure Asset and is protected during runtime |
|                           | 1                            | 0x50000000        | 0x53600000      | All AIDs can access                                 | 1      | 1      | 1      | 1      | 1      | 1      | 1      | 1         | To access peripheral region master side MPU |
| FW R5SS1\_CORE1\_AHB\_MST | 0                            | 0x53600000        | 0x53600400      | HSM                                                 | 1      | 1      | 1      | 1      | 1      | 1      | 1      | 1         | Secure Asset and is protected during runtime |
|                           | 1                            | 0x50000000        | 0x53600000      | All AIDs can access                                 | 1      | 1      | 1      | 1      | 1      | 1      | 1      | 1         | To access peripheral region master side MPU |


\endcond


### Resources available
By default, the access to the crypto resources are firewalled on HS-FS devices.
The HSM firmware so loaded, bypasses the firewalls and makes the following
crypto modules available for the R5FSS to use.

* AES (public context)
* SHA (public context)

This provides ability for the R5F core to be able to do the following
computations:

* AES encryption/decryption
* SHA/HMAC hash calculation

SBL should always wait for `HSM_MSG_BOOT_NOTIFY` before using the
crypto accelerator because HSM firmware initializes these firewalls.
Failing to do so, the SBL or application may run into abort exception while
accessing the MMR regions for the crypto accelerator.

### Services

The TIFS-MCU firmware that gets loaded on HSM provides the variety of services. For more
information refer @ref DRIVERS_HSMCLIENT_PAGE

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

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
- Cryptography Modules
    - \subpage DRIVERS_DTHE_PAGE (Data Transform and Hashing Engine)
        - Description of DTHE architecture and APIs available to use AES and SHA engine.

\note EDMA support for AES and SHA will be added in future releases. Currently,
the drivers only operate in CPU mode of data copy.
\endcond

\cond SOC_AM273X
- Cryptography Modules
    - \subpage DRIVERS_DTHE_PAGE (Data Transform and Hashing Engine)
        - Description of DTHE architecture and APIs available to use AES and SHA engine.

\note EDMA support for AES and SHA will be added in future releases. Currently,
the drivers only operate in CPU mode of data copy.

\endcond