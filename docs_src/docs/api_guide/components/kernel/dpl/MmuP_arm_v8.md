# MMU for ARMv8 (ARM A53) {#KERNEL_DPL_MMU_ARMV8_PAGE}

[TOC]

## Features Supported

- APIs to setup MMU regions, including region address, size, attributes like access permissions, cache properties
- APIs to enable, disable MMU
- API to check if MMU is enabled

## Features NOT Supported

NA

## Important Usage Guidelines

- MMU regions can overlap each other, with higher numbered regions taking more precedence in case of address overlaps.

## Example Usage

Include the below file to access the APIs,
\snippet MmuP_arm_v8_sample.c include

Example to setup MMU regions and enable MMU is shown below,
\snippet MmuP_arm_v8_sample.c mmu

## API

\ref KERNEL_DPL_MMU_ARMV8
