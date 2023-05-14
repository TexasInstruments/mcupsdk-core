# SENT {#SENT}

[TOC]

## Introduction

The SENT receiver firmware running on PRU-ICSS provides an interface to execute the SENT protocol. The SENT application interacts with the SENT receiver firmware interface. The SENT firmware and examples are based on GPIO interface from PRU-ICSS.

# Feature Supported

| Feature                            | Support | Remarks                                               |
| ---------------------------------- | ------- | ----------------------------------------------------- |
| No. of Channels                    | 8       | Can support 1 to 8 channels                           |
| Min Tick Period                    | 500ns   | Experimental support                                  |
| Sync Correction                    | Yes     | 20% tolerance value                                   |
| Frame buffering                    | No      | Supports only Single frame buffering                  |
| Configurable FIFO                  | No      | Will be implemented in future release                 |
| Receiving 1 – 6 Data Nibbles       | Yes     | Receiving and decoding Supported in Firmware          |
| CRC calculation                    | Yes     | CRC calculation done along with data nibble reception |
| Pause Pulse                        | No      | Will be implemented in future release                 |
| Short Serial Message Format        | No      | Will be implemented in future release                 |
| Enhanced Serial Message Format     | No      | Will be implemented in future release                 |
| Successive Calibration Pulse Check | No      | Will be implemented in future release                 |
| Selectable data length for receive | No      | Will be implemented in future release                 |

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

SysConfig can be used to configure things mentioned below:

- Syscfg based customization will be supported in future releases.

## SENT Design

\subpage SENT_DESIGN explains the firmware design in detail.

## Example

- \ref EXAMPLES_SENT_DECODER
