# CPUID for ARMv7 (ARM R5) {#KERNEL_DPL_CPUID_PAGE}

[TOC]

## Features Supported

- APIs to get R5F Cluster Id and Core Id.

## Features NOT Supported

NA

## Important Usage Guidelines

\cond !SOC_AM62X
- Refer to ARMv7-R architecture manual for more details
- API should be used in priviledge mode only.
\endcond

## Example Usage

Include the below file to access the APIs,
\snippet CpuIdP_sample.c include

Example to get R5F Cluster Id and Core Id,
\snippet CpuIdP_sample.c cpuIdP

## API

\ref KERNEL_DPL_CPU
