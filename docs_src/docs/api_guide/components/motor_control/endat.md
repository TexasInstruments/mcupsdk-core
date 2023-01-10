# EnDat {#ENDAT}

[TOC]

## Introduction

EnDat is a bidirectional interface for position encoders. During EnDat operation the EnDat receiver receives position information from the EnDat position encoder.

## Features Supported

   -  EnDat 2.2 command set
   -  EnDat 2.1 command set
   -  Interrupted and continuous clock mode
   -  Cable length up to 100m @8MHz
   -  Propagation delay compensation (capable of handling different propagation delay of different
      propagation delay of different channels in concurrent multi
      channel configuration)
   -  Automatic estimation of propagation delay
   -  Receive on-the-fly CRC verification of position, parameters and additional information
   -  Two modes of operation - host trigger and periodic trigger
   -  Channel select
   -  Concurrent multi channel support (up-to 3 encoders with identical part number @ 8MHz maximum)
   -  "Multi Channel with Encoders of Different Make" using load share mode (Each of PRU, RTU-PRU, and TX-PRU from one PRU-ICSSG slice handles one channel)
   -  Safety Readiness: Recovery time

## Features Not Supported

In general, peripherals or features not mentioned as part of "Features Supported" section are not
supported in this release, including the below
-  Safety
-  Clock configuration up to 16MHz
-  Independent clocks on multi channel mode.

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

SysConfig can be used to configure things mentioned below:
- Selecting the ICSSG instance. (Tested on ICSSG0)
- Selecting the ICSSG0PRUx instance.(Tested on ICSSG0-PRU1)
- Configuring PINMUX.
- Channel selection.
- Selecting Multi Channel with Encoders of Different Make" using load share mode.


## ENDAT Design

\subpage ENDAT_DESIGN explains the design in detail.

## Example
\ref EXAMPLE_MOTORCONTROL_ENDAT

## API
\ref ENDAT_API_MODULE

