# Third Party Protocol Stacks or Customers using their own Stacks {#INDUSTRIAL_COMMS_3P_STACK}

[TOC]


In this model, the adaptation of stack to PRU-ICSS on TI SoCs needs to be done by third party vendors or customers themselves. SDK provides the PRU-ICSS firmware for protocols and the hardware abstraction layer APIs to communicate with thr firmware.

## Industrial Communications FWHAL {#INDUSTRIAL_COMMS_FWHAL}

FWHAL(Firmware and Hardware Abstraction Layer) implements the key interface between protocol specific PRU-ICSS firmware and protocol stack. The PRU-ICSS firmware runs on the PRU cores, offloading the time-critical link layer processing from the main ARM processor, running FreeRTOS. The FW HAL provides simple access to the PRU-ICSS resources and integrates easily with the protocol stack and application software running on the ARM core.

FWHAL for following industrial communication protocols are present:

- \subpage ETHERCAT_SUBDEVICE_FWHAL
\cond SOC_AM64X || SOC_AM243X
- \subpage ETHERNETIP_ADAPTER_FWHAL
- \subpage PROFINET_DEVICE_FWHAL
- \subpage HSR_PRP_FWHAL
\endcond

## Examples

- EtherCAT SubDevice : \ref EXAMPLES_INDUSTRIAL_COMMS_ETHERCAT_SLAVE_BECKHOFF_SSC_DEMO example for EtherCAT SubDevice is based on Beckhoff SSC. The stack sources should be added manually and patched to build this example. Files for adapting the stack to \ref ETHERCAT_SUBDEVICE_FWHAL are present in `${SDK_INSTALL_PATH}/source/industrial_comms/ethercat_slave/beckhoff_stack/stack_hal` folder.
\cond SOC_AM64X || SOC_AM243X
- HSR/PRP(High Availability Seamless Redundancy/Parallel Redundancy Protocol) : \ref EXAMPLES_INDUSTRIAL_COMMS_HSR_PRP_DEMOS
\endcond

\cond SOC_AM64X || SOC_AM243X

\note The work-around for issue "i2329 - MDIO: MDIO interface corruption (CPSW and PRU-ICSS)" (described in <a href="https://www.ti.com/lit/er/sprz457e/sprz457e.pdf">AM64x/AM243x Processor Silicon Revision 1.0, 2.0 (Rev. E)</a>) is available in this release. Please see \ref INDUSTRIAL_COMMS_MDIO_MANUALMODE_FW_USAGE for more details.

\endcond
