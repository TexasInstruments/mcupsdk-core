# HWA {#DRIVERS_HWA_PAGE}

[TOC]

The HWA driver provides APIs to configure, trigger and obtain results from the
hardware accelerator

## Features Supported

- All HWA operating modes: FFT, CFAR, compression/decompression and local maxima
- API to program common HWA registers, HWA param sets
- API to program pre/post processing blocks and core computational units
- API to get output statistics and histogram outputs
- Interrupt and polling mode of operations - param done, processing done
- Manual or DMA trigger for data copy from/to HWA memories

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

- HWA instances selection
- Init/Deinit API integration with SYSCFG

## Features NOT Supported

- None

## Important Usage Guidelines

- The HWA Driver needs to be initialized once across the System. This is
  done using the #HWA_init. None of the HWA API can be used without invoking
  this API. Note: This API is integrated into Sysconfig and can be skipped
  by user if Sysconfig is used for integrating HWA
- Once the HWA Driver has been initialized; the HWA Driver instance can be opened
  using the #HWA_open. The #HWA_open can be called multiple times from different
  context to obtain a valid HWA handle. However, only the first call to #HWA_open will
  perform the hardware initialization. Other subsequent calls will just return the already
  open handle.

## Example Usage

Include the below file to access the APIs
\snippet Hwa_sample.c include

Instance Open Example
\snippet Hwa_sample.c open

Instance Close Example
\snippet Hwa_sample.c close

Instance Processing Example
\snippet Hwa_sample.c process

## API

\ref DRV_HWA_MODULE
