# MPU FIREWALL {#DRIVERS_MPU_FIREWALL_PAGE}

[TOC]

MPU FIREWALL driver provides API to read different regions in all supported firewalls.

## Features Supported

- API to read MPU Firewall regions, including region start address, size, attributes like access permissions and AID configurations
- APIs to read fault address, fault status and to get interrupt status for a mpu region.


## Features NOT Supported

- Configuration of MPU firewall regions is not supported via this driver and is supported via HSM Services. For more information \ref DRIVERS_HSMCLIENT_SET_FIREWALL

## Important Usage Guidelines

- None

## Example Usage

Include the below file to access the APIs
\snippet MpuFirewall_sample.c include

Example to setup MPU Firewall regions is shown below,
\snippet MpuFirewall_sample.c mpu

## API

\ref DRV_MPU_FIREWALL_MODULE