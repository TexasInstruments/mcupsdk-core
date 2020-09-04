# ADCBUF {#DRIVERS_ADCBUF_PAGE}

[TOC]

The ADCBUF in the AWR29xx SoCs can be programmed at a high-level using
the ADCBUF driver software that allows hook-up with an operating system.
The ADCBUF driver exposes programming of most features provided by the IP.

The ADCBuf driver samples an analog waveform at a specified frequency.
The resulting samples are transferred to a buffer provided by
the application. The driver can either take n samples once, or continuously
sample by double-buffering and providing a callback to process each finished
buffer.

## Features Supported

- Source selection: DFE or HIL
- Set chirp thresholds for ping and pong buffers
- Enable or disable continuous mode
- Start and stop continuous mode of operation
- Configure data format, chirp quality and test pattern
- Enable or disable channel

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

- ADCBUF instances selection

## Features NOT Supported

- None

## Important Usage Guidelines

- None

## Example Usage

Include the below file to access the APIs
\snippet Adcbuf_sample.c include

Open ADCBUF
\snippet Adcbuf_sample.c open_adcbuf

Close ADCBUF
\snippet Adcbuf_sample.c close_adcbuf

## API

\ref DRV_ADCBUF_MODULE
