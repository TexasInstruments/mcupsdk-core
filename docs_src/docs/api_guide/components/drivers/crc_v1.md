# CRC {#DRIVERS_CRC_PAGE}

[TOC]

CRC (Cyclic Redundancy Check) driver provide API to perform CRC operation

## Features Supported

- Two channels
- 8, 16, 32 and 64 bit data size
- CPU mode of operations

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

- CRC instances selection

## Features NOT Supported

- DMA mode of operation

## Important Usage Guidelines

- None

## Example Usage

Include the below file to access the APIs
\snippet Crc_sample.c include

CRC Module Initialization Example
\snippet Crc_sample.c init

CRC Module Configuration Example
\snippet Crc_sample.c config_v1

CRC Operation Example
\snippet Crc_sample.c operation_v1

## API

\ref DRV_CRC_MODULE
