# FSI {#DRIVERS_FSI_PAGE}

[TOC]

The Fast Serial Interface (FSI) driver provides API to program the FSI module.
FSI is a serial communication peripheral capable of reliable and robust high-speed communications.
FSI is used to transfer/receive data frame with several available configurations e.g frame size, bandwidth etc, signaling events completion via interrupts or status registers.

## Features Supported

- Supports programmable packet data length of 1-to-16 words where each word consists of 16-bits
- Supports programmable frame tagging (0000-to-1111) for message filtering
- Supports programmable data-lines( 1-or-2) for communication
- Supports Double Data Rate (DDR) communication
- Supports programmable interrupts via 2 interrupt lines per FSI core
\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
- Supports DMA mode. The DMA event will only be generated for data frames.
- Supports programmable TX delay line control
\endcond

## Features NOT Supported

- Multi-Peripheral configuration
- clkstop_req/ack functionality. Software must ensure that the FSI IP's on both host and remote device are idle before triggering clkstop_req.

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

- Selection of instances present in the device
- Configuring pinmux based on selected instance and pin configuration
\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
- Interrupt Crossbar configuration for FSI RX and TX.
\endcond

## Important Usage Guidelines

- Note: FSI-RX and FSI-TX are completely independent and separate module.
For communication, both needs to be configured separately.
\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
- Interrupt Crossbar configuration should be done to route FSI RX and TX interrupts if SysCfg is not used.
\endcond

## Example Usage

Include the below file to access the APIs
\snippet Fsi_sample.c include

FSI tx config
\snippet Fsi_sample.c FSI_tx_config

FSI rx config
\snippet Fsi_sample.c FSI_rx_config

FSI configuration to enable interrupt
\snippet Fsi_sample.c FSI_interrupt
## API

\ref DRV_FSI_MODULE
