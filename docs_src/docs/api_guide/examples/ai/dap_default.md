# DAP Default {#EXAMPLES_AI_DAP_DEFAULT}

[TOC]

# Introduction

This example demonstrates the Device Agent Protocol (DAP) communication layer used for integration with Edge AI Studio. DAP provides a standardized UART-based protocol for streaming sensor data, receiving commands, and transmitting inference results between the microcontroller and Edge AI Studio.

This example serves as a reference implementation for:
- Establishing DAP communication with Edge AI Studio
- Implementing sensor data streaming
- Handling pipeline configuration commands
- Transmitting inference results

# Supported Combinations {#EXAMPLES_AI_DAP_DEFAULT_COMBOS}

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos      | 
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/ai/dap_default/

\endcond

\cond SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos      | 
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/ai/dap_default/

\endcond

\cond SOC_AM261X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos      | 
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/ai/dap_default/

\endcond

# DAP Protocol Overview

The Device Agent Protocol (DAP) provides:
- **Frame-based communication**: CRC-validated frames for reliable data transfer
- **Multiple data channels**: Support for sensor signals, inference signals, results, values, and logs
- **Command handling**: Pipeline configuration and control commands from Edge AI Studio
- **Multi-sensor streaming**: Support for streaming data from multiple sensors simultaneously

# Key DAP Functions

| Function | Description |
|----------|-------------|
| `Dap_Init()` | Initialize DAP instance |
| `Dap_Process()` | Main processing loop |
| `Dap_SendSensorData()` | Stream sensor data |
| `Dap_StartSensorStream()` | Start sensor streaming |
| `Dap_StopSensorStream()` | Stop sensor streaming |

# Dependencies

- Edge AI Studio for communication
- UART interface for DAP protocol

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- Connect Edge AI Studio to the device via UART

# See Also

\ref EXAMPLES_AI

# Sample Output

\code
[DAP] Example started...
[DAP] Initialized successfully. Waiting for commands...
\endcode
