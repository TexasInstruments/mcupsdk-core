# Industrial Communications Toolkit {#INDUSTRIAL_COMMS}

[TOC]

## Introduction

The Industrial Communications Toolkit enables real-time industrial communications for TI processors. Industrial communication is typically handled by the Programmable Real-Time Unit Industrial Communication Subsystem (PRU-ICSS). The PRU-ICSS is a co-processor subsystem containing Programmable Real-Time (PRU) cores and Ethernet media access controllers (EMACs), which implement the low level industrial Ethernet and fieldbus protocols through firmware. The upper layers of the protocols stacks are implemented in software running on Arm cores.

PRU cores are primarily used for industrial communication, and can also be used for other applications such as motor control and custom interfaces. The PRU-ICSS frees up the main Arm cores in the device for other functions, such as control and data processing.

Three models for protocol software are supported. In the first model, full protocol stacks are provided by TI and already integrated and certified with PRU-ICSS firmware on TI hardware. For the other two models, TI provides PRU-ICSS firmware and documentation to allow customers to integrate their own stack or to engage with a third party for stack integration.

\cond SOC_AM263X
\attention Full Protocol Stacks from TI is under development and not available in this release.
\endcond

- \subpage INDUSTRIAL_COMMS_TI_STACK : Stack and PRU-ICSS firmware from TI
- \subpage INDUSTRIAL_COMMS_3P_STACK : PRU-ICSS firmware and associated drivers from TI

\cond SOC_AM64X || SOC_AM243X

\note The work-around for issue "i2329 - MDIO: MDIO interface corruption (CPSW and PRU-ICSS)" (described in <a href="https://www.ti.com/lit/er/sprz457e/sprz457e.pdf">AM64x/AM243x Processor Silicon Revision 1.0, 2.0 (Rev. E)</a>) is available in this release. Please see \subpage INDUSTRIAL_COMMS_MDIO_MANUALMODE_FW_USAGE for more details.

\endcond


## Additional References {#INDUSTRIAL_COMMS_ADDITIONAL_REFERENCES}

<table>
<tr>
    <th>Document
    <th>Description
</tr>
<tr>
    <td>[Industrial Communication Protocols Supported on
Sitara Processors](https://www.ti.com/lit/pdf/sprach6)
    <td>Application Report by TI describing the industrial communication protocols supported on Sitara Processors.
</tr>
<tr>
    <td>[EtherCAT on Sitara Processors](https://www.ti.com/lit/pdf/spry187)
    <td>Application note by TI on the EtherCAT SubDevice implementation on Sitara Processors.
</tr>
\cond SOC_AM243X || SOC_AM273X || SOC_AM64X || SOC_AWR294X
<tr>
    <td>[EtherNet/IP on TI's Sitara processors](https://www.ti.com/lit/pdf/spry249)
    <td>Application note by TI on the EtherNet/IP Adapter implementation on TI's Sitara Processors.
</tr>
<tr>
    <td>[PROFINET on TI's Sitara processors](https://www.ti.com/lit/pdf/spry252)
    <td>Application note by TI on the Profinet implementation on TI's Sitara Processors.
</tr>
\endcond
<tr>
    <td>[Ethernet PHY Configuration Using MDIO for Industrial Applications](https://www.ti.com/lit/an/spracc8/spracc8.pdf)
    <td>This document intends to expedite the development of industrial Ethernet applications on custom boards with migration and troubleshooting guides for the Ethernet PHYs.
</tr>
</table>
