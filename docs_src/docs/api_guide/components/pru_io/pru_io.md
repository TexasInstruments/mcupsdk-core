# PRU IO {#PRU_IO}

[TOC]

## Introduction

The Sitara Devices contain PRU Cores which are capable of executing instructions in a completely deterministic manner. The various clock speed options available are 200MHz, 225MHz, 250MHz, 300MHz, 333MHz giving us a resolution of upto 3ns. This allows us to use PRU Cores for realtime I/O control applications. The modules present in this section provides support for quicker development of software for such use-cases.

This section consists of modules providing support for PRU cores:

\cond SOC_AM64X || SOC_AM243X
- \subpage COMPONENT_PRU_GUIDE
- \subpage DRIVERS_PRU_ADC
- \subpage DRIVERS_PRU_IPC
\endcond
