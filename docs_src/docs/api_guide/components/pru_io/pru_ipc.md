# PRU IPC {#DRIVERS_PRU_IPC}

[TOC]

PRU IPC module provides APIs for communicating with the PRU core with low latency. The communication is done in the form of blocks of data transfer in one exchange. Options are available for configuring multiple buffers and multiple blocks per buffer. The PRU sends one block of data for all buffers at a time, and the next block after that.

The block diagram for the PRU IPC communication is given below.

  \imageStyle{pru_ipc_arch.png,width:65%}
  \image html pru_ipc_arch.png " "
## Features Supported

- Configurables: number of buffers, number of blocks, block size, data size
- Interrupt option after a block of data is sent by PRU
- Configurable shared memory location where the buffers will be placed - by specifying it in linker file
- Optional interrupt to PRU after a block of data is sent by R5F
- Custom interrupt mapping through ICSS INTC module

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

SysConfig is used for configuring all the PRU IPC options mentioned above, through PRU IPC module under PRU IO.

- Based on the configuration selected, the SysConfig generated code does following:
    - Create macros like `CONFIG_PRU_IPC1` using the name passed in SysConfig. This is used as an input to \ref PRU_IPC_open API.
    - Generates sample code for linker in `ti_drivers_config.c`

## Features NOT Supported

- Currently only PRU cores are supported (not RTU PRU and TX PRU cores)

## Usage Guidelines

### Add PRU IPC using SysConfig

- Add a PRU IPC instance using SysConfig, by selecting `PRU IPC` under `PRU IO` in the left pane in SysConfig
- Configure the instance as per requirements

### Update linker command file {#DRIVERS_PRU_IPC_LINKER}

- A shared memory region between CPU and PRU-ICSS needs to be allocated which the PRU and R5F cores will use for exchanging data. The generated code in `ti_drivers_config.c` contains sample linker file code which needs to be placed in project's linker file.

  \imageStyle{pru_ipc_linker.png,width:80%}
  \image html pru_ipc_linker.png " "

### Update MPU for the CPU

- The shared memory section (described in the above section) which was put in the linker command file needs to be mapped as `Cached+Sharable` for the CPU.
- This can be done via SysConfig, by adding additional MPU entries using the `MPU` module under `TI DRIVER PORTING LAYER (DPL)` in SysConfig. Following is an example:

  \image html pru_ipc_mpu_config.png


These APIs are for the R5F side. Please also look into the PRU side macros available for usage in PRU firmware.

## Example Usage

The PRU IPC is being used in PRU-ADC interfacing examples in which PRU transfers the samples to R5F (to TCM). There,
Number of channels correspond to number of buffers and packet size correspond to sample size.

Initialize PRU IPC Instance
\snippet Pru_ipc_sample.c init_pruIpc

Transfer data
\snippet Pru_ipc_sample.c transferData


Usage on PRU Firmware Side:
\code
    m_pru_ipc_init      R0.b0           ; Initializes PRU IPC paramenters, use R0.b0 register only
    ldi                 R1.b0, &R3      ; Starting memory address of registers to send
    m_pru_ipc_send      R1.b0, R3, R0.b0, PRU_IPC_RX_INTR_ENABLE, PRU_IPC_RX_EVENT  ; Send data to shared memory
\endcode

## API

\ref DRV_PRU_IPC_MODULE
