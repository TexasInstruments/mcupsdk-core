#  Industrial Communications Toolkit {#EXAMPLES_INDUSTRIAL_COMMS}

Following is the list of all the examples related to industrial communication protocols:

\cond SOC_AM64X || SOC_AM243X
- EtherCAT SubDevice
    - \subpage EXAMPLES_INDUSTRIAL_COMMS_ETHERCAT_SLAVE_DEMOS : Evaluation example of pre-integrated stack.
    - \subpage EXAMPLES_INDUSTRIAL_COMMS_ETHERCAT_SLAVE_BECKHOFF_SSC_DEMO : Example based on Beckhoff SSC. The stack sources should be added manually and patched to build this example.
- EtherNet/IP Adapter
    - \subpage EXAMPLES_INDUSTRIAL_COMMS_ETHERNETIP_ADAPTER_DEMOS : Evaluation example of pre-integrated stack.
- IO-Link Controller
    - \subpage EXAMPLES_INDUSTRIAL_COMMS_IOLINK_MASTER_DEMO : Evaluation example of pre-integrated stack.
- PRP (Parallel Redundancy Protocol)
    - \subpage EXAMPLES_INDUSTRIAL_COMMS_HSR_PRP_DEMOS : Evaluation example.
- EtherCAT-IOLink Gateway
    - \subpage EXAMPLES_INDUSTRIAL_COMMS_ETHERCAT_IOLINK_GATEWAY_DEMO : Evaluation example of pre-integrated stack.

\note
  Starting with MCU+ SDK version 08.04.00, the existing PROFINET RT stack and examples will no longer be available in the SDK. For more information, see \ref INDUSTRIAL_COMMS_TI_STACK_PROFINET_STACK_TRANSITION.

- HSR/PRP (High Availability Seamless Redundancy/Parallel Redundancy Protocol)
    - \subpage EXAMPLES_INDUSTRIAL_COMMS_HSR_PRP_DEMOS : Evaluation example.
\endcond

\cond SOC_AM263X
- EtherCAT SubDevice
    - \subpage EXAMPLES_INDUSTRIAL_COMMS_ETHERCAT_SLAVE_BECKHOFF_SSC_DEMO : Example based on Beckhoff SSC. The stack sources should be added manually and patched to build this example.
\endcond