# MCASP {#DRIVERS_MCASP_PAGE}

[TOC]

The Multi Channel Audio Seiral Peripheral (MCASP) driver is a generic,
full-duplex driver that transmits and receives data using the MCASP interface.
This is specially designed to transmit and receive the digital audio data.

## Features Supported

- Interrupt mode operation
- DMA mode operation
- Internal Loopback
- Transmit and Receive in TDM format with multi-serializer, multi-slot support.
- Different audio buffer formats for application
- SysConfig support for driver parameter configuration and initialization.

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

- Note: Currently, some register configurations are supported through SysConfig,
while for others, user needs to use hardcoded McASP register configurations
defined in the structure "gMcaspAttrs" located in:
<mcasp_loopback\am273x-evm\c66ss0_nortos\ti-c6000\generated\ti_drivers_config.c>
manually by referring the register descriptions in the device TRM.

## Features NOT Supported

- Transmit in DIT mode
- Burst Mode

## Usage Overview

### API Sequence

To use the MCASP driver to send data over the MCASP interface, the application
calls the following APIs:

- #MCASP_init(): Initialize the MCASP driver.
- #MCASP_open():  Open an instance of the MCASP driver
- #MCASP_submitRx(): Submit buffers to MCASP driver for reception
- #MCASP_submitTx(): Submit buffers to MCASP driver for transmission
- #MCASP_startTransferRx(): Start MCASP receive.
- #MCASP_startTransferTx(): Start MCASP transmit.
- #MCASP_stopTransferRx(): Stop MCASP receive.
- #MCASP_stopTransferTx(): Stop MCASP transmit.
- #MCASP_withdrawRx(): withdraw buffers submitted to driver for reception
- #MCASP_withdrawTx(): withdraw buffers submitted to driver for transmission
- #MCASP_close():  De-initialize the MCASP instance.
- #MCASP_deinit(): De-Initialize the MCASP driver.

### Supported Application Audio Buffer Formats

The McASP driver provides various buffer formats for sending the data to/from
the audio device. The differences between these formats arise from the way the
audio samples are collected from various serializers and their timeslots and
arranged in the system’s memory. This way the application can choose to run
audio processing algorithms over the data without the need for re-arranging
those data every frame. This section provides an overview of the various formats
. In the explanatory diagrams in each section, McASP controller recieves samples
 in frame intervals denoted by t1,t2..tn. The McASP driver collects these and
arranges those samples in the memory in various formats in to the host’s System
 memory. We have chosen 32 bit samples and 32-bit word addresses throughout for
 simplicity.

- **1-Serializer Multi-Slot Interleaved**:
  This is applicable if multiple slots are used with one serializer. The samples
  from the different timeslots are stored interleaved in the memory as below.
  Different timeslots’ samples are denoted by different colors and are labelled
  t1,t2..tn according to the time they arrive at the serializer.

  <img src="mcasp_buff_format_1serMultiSlotInterleaved.PNG" width="700" height="400" />

- **1-Serializer Multi-Slot NonInterleaved**:
  This is applicable if multiple slots are used with one serializer. The samples
  from the different timeslots are grouped together on the basis of the timeslot
  and stored in the memory as shown below. Different timeslots’ samples are
  denoted by different colors and are labelled t1,t2..tn according to the time
  they arrive at the serializer.

  <img src="mcasp_buff_format_1serMultiSlotNonInterleaved.PNG" width="700" height="400" />

- **Multi-Serializer Multi-Slot Interleaved Type1**:
  This is applicable if multiple serializers are used and each serializer
  containing multiple timeslots. The samples are stored in the memory interleaved
  based on serializer and timeslots as shown below. In this example, there are 3
  serializers and 2 timeslots per serializers whose samples are noted by Ln (left)
  and Rn (right). Different serializers’ samples are denoted by different colors.

  <img src="mcasp_buff_format_MultiSerMultiSlotType1.PNG" width="700" height="400" />


- **Multi-Serializer Multi-Slot Interleaved Type2**:
  This is applicable if multiple serializers are used and each serializer containing
  multiple timeslots. The samples are grouped based on the serializer and within
  one serializer, the timeslots are interleaved as shown below. In this example,
  there are 3 serializers and 2 timeslots per serializers whose samples are noted
  by Ln (left) and Rn (right).Different serializers’ samples are denoted by different
  colors.

  <img src="mcasp_buff_format_MultiSerMultiSlotType2.png" width="700" height="400" />

### Loopjob Configuration

Mcasp driver allows loopjob enabled/disabled configuration to handle delayed application buffer submission at run time.
Loopjob is a default buffer that gets transmitted / received into if the application fails to submit the buffers in time.
When loopjob is disabled the last 2 application buffers will be re programmed and re used.
The application callback corresponding to the re programmed buffers is not called.

- Loopjob buffer is programmed in sys config. Sys config will define a uint8_t array and initialize it to 0.

- Application can access the buffer and optionally initialize the data.

- If the loopjob buffer is programmed, after sending the full loopjob buffer next submitted buffer is programmed.


### Buffer Queuing

- The application buffers need to be programmed before starting the transfer.

- If the loopjob is enabled application can start without queuing any buffers, Driver will work with the loopjob buffers till application queues the actual buffers.

- If loopjob is disabled at least 2 buffers should be queued before starting the mcasp transfer.
New buffer will be programmed in the interrupt callback corresponding to current transfer. the application is expected to submit the new buffer before that else the same buffer is re programmed, so it is recommended to start the transfer with at least 3 buffers queued.

## Example Usage

Include the below file to access the APIs
\snippet Mcasp_sample.c include

Instance Open Example
\snippet Mcasp_sample.c open

Instance Close Example
\snippet Mcasp_sample.c close

Start McASP Transfer Example
\snippet Mcasp_sample.c start_transfer_loopback

Stop McASP Transfer Example
\snippet Mcasp_sample.c stop_transfer_loopback


## API

\ref DRV_MCASP_MODULE
