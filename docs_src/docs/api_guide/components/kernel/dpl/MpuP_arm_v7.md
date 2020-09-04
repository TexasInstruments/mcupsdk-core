# MPU for ARMv7 (ARM R5, ARM M4) {#KERNEL_DPL_MPU_ARMV7_PAGE}

[TOC]

## Features Supported

- APIs to setup MPU regions, including region address, size, attributes like access permissions, cache properties
- APIs to enable, disable MPU
- API to check if MPU is enabled

## Features NOT Supported

NA

## Important Usage Guidelines

\cond !SOC_AM62X
- Refer to ARMv7-R or ARMv7-M architecture manual for more details
- The number of MPU entries varies based on the ARM R5, ARM M4 instantiated in a given DEVICE, refer DEVICE datasheet for more details.
- In R5 for this DEVICE, there are 16 MPU regions
\endcond
\cond SOC_AM62X
- Refer to ARMv7-M architecture manual for more details
- The number of MPU entries varies based on the ARM M4 instantiated in a given DEVICE, refer DEVICE datasheet for more details.
\endcond
- In M4 for this DEVICE, there are 16 MPU regions
- MPU regions can overlap each other, with higher numbered regions taking more precedence in case of address overlaps.

## Example Usage

Include the below file to access the APIs,
\snippet MpuP_arm_v7_sample.c include

Example to setup MPU regions and enable MPU is shown below,
\snippet MpuP_arm_v7_sample.c mpu

## API

\ref KERNEL_DPL_MPU_ARMV7
