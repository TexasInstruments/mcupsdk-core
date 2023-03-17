# Motor Control {#ENCODERS_DOC}

[TOC]

## Introduction
\cond SOC_AM64X || SOC_AM243X
The Motor Control-Encoder Toolkit enables real-time communications with encoders for TI processors/MCUs. Communication is typically handled by the Programmable Real-Time Unit Industrial Communication Subsystem (PRU-ICSS). The PRU-ICSS is a co-processor subsystem containing Programmable Real-Time (PRU) cores which implement the low level firmware. The upper application layer are implemented in software running on Arm cores.

PRU cores are primarily used for communication with encoder, and can also be used for other applications such as motor control and custom interfaces. The PRU-ICSS frees up the main Arm cores in the device for other functions, such as control and data processing.

Applications and PRU-ICSS firmware to communicate with following encoders are provided in the SDK:

- \subpage ENDAT
- \subpage HDSL
- \subpage TAMAGAWA
\endcond

\cond SOC_AM263X
The Motor Control-Encoder Toolkit enables real-time communications with encoders for TI processors/MCUs.
- \subpage TAMAGAWA_OVER_UART
\endcond