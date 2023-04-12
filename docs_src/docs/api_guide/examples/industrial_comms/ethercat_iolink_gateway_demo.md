# IO-Link Controller Demo{#EXAMPLES_INDUSTRIAL_COMMS_ETHERCAT_IOLINK_GATEWAY_DEMO}

[TOC]

# Introduction

This example is an EtherCAT-IOLink Gateway application based on evaluation stack provided in the SDK. Links to important sections in the detailed documentation are provided below:

\cond SOC_AM64X

- \htmllink{../industrial_protocol_docs/am64x/ethercat_iolink_gateway/index.html, Home Page}
- \htmllink{../industrial_protocol_docs/am64x/ethercat_iolink_gateway/ethercat_datasheet.html, Data Sheet}
- \htmllink{../industrial_protocol_docs/am64x/ethercat_iolink_gateway/releasenotes.html, Release Notes}
- \htmllink{../industrial_protocol_docs/am64x/ethercat_iolink_gateway/usergroup0.html, Example Quick Start}
- \htmllink{../industrial_protocol_docs/am64x/ethercat_iolink_gateway/modules.html, API Documentation}

\endcond

\cond SOC_AM243X

- \htmllink{../industrial_protocol_docs/am243x/ethercat_iolink_gateway/index.html, Home Page}
- \htmllink{../industrial_protocol_docs/am243x/ethercat_iolink_gateway/ethercat_datasheet.html, Data Sheet}
- \htmllink{../industrial_protocol_docs/am243x/ethercat_iolink_gateway/releasenotes.html, Release Notes}
- \htmllink{../industrial_protocol_docs/am243x/ethercat_iolink_gateway/usergroup0.html, Example Quick Start}
- \htmllink{../industrial_protocol_docs/am243x/ethercat_iolink_gateway/modules.html, API Documentation}

\endcond

\attention If you need to reload and run the example again, EVM power-cycle is MUST.

# Supported Combinations

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ICSSG          | ICSSG0 and ICSSG1
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/industrial_comms/ethercat_iolink_gateway_demo

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ICSSG          | ICSSG0 and ICSSG1
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER (E1/E2 Revision)
 Example folder | examples/industrial_comms/ethercat_iolink_gateway_demo

\endcond
