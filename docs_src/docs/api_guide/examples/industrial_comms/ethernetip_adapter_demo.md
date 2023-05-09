# EtherNet/IP Adapter Demos{#EXAMPLES_INDUSTRIAL_COMMS_ETHERNETIP_ADAPTER_DEMOS}

[TOC]

# Introduction

These examples are EtherNet/IP Adapter applications based on evaluation stacks provided in the SDK. Links to important sections in the detailed documentation are provided below:

\cond SOC_AM64X

- \htmllink{../industrial_protocol_docs/am64x/ethernetip_adapter/index.html, Home Page}
- \htmllink{../industrial_protocol_docs/am64x/ethernetip_adapter/eip_datasheet.html, Data Sheet}
- \htmllink{../industrial_protocol_docs/am64x/ethernetip_adapter/eip_releasenotes.html, Release Notes}
- \htmllink{../industrial_protocol_docs/am64x/ethernetip_adapter/eip_quickstart.html, Example Quick Start}
- \htmllink{../industrial_protocol_docs/am64x/ethernetip_adapter/modules.html, API Documentation}

\endcond

\cond SOC_AM243X

- \htmllink{../industrial_protocol_docs/am243x/ethernetip_adapter/index.html, Home Page}
- \htmllink{../industrial_protocol_docs/am243x/ethernetip_adapter/eip_datasheet.html, Data Sheet}
- \htmllink{../industrial_protocol_docs/am243x/ethernetip_adapter/eip_releasenotes.html, Release Notes}
- \htmllink{../industrial_protocol_docs/am243x/ethernetip_adapter/eip_quickstart.html, Example Quick Start}
- \htmllink{../industrial_protocol_docs/am243x/ethernetip_adapter/modules.html, API Documentation}

\endcond

\attention If you need to reload and run the example again, EVM power-cycle is MUST.

\note The work-around for issue "i2329 - MDIO: MDIO interface corruption (CPSW and PRU-ICSS)" (described in <a href="https://www.ti.com/lit/er/sprz457e/sprz457e.pdf">AM64x/AM243x Processor Silicon Revision 1.0, 2.0 (Rev. E)</a>) is not available for this example in this release. It will be available in next release.

# Supported Combinations

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ICSSG          | ICSSG1
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/industrial_comms/ethernetip_adapter_demo

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ICSSG          | ICSSG1
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER (E1/E2 Revision)
 Example folder | examples/industrial_comms/ethernetip_adapter_demo

\endcond

\note This example will not work on PG2.0 HS-FS devices.

