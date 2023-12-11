#  HSM Services Example {#EXAMPLES_HSM_SERVICES}

[TOC]

# Introduction

This example demonstrates the use of HSM services.

Three services are available.
1. UID is a unique identifier for each device. It can be extracted  using **get uid service**.
2. HSM **get version service** serves the client with TIFS-MCU version information.
3. HSM **set firewall service** is used to configure MPU firewall regions.

# Supported Combinations {#EXAMPLES_HSM_SERVICES_COMBOS}

\cond SOC_AM263X || SOC_AM263PX || SOC_AM273X

 Parameter             | Value
 ----------------------|-----------
 CPU + OS              | r5fss0_0 nortos
 Toolchain             | ti-arm-clang
 Boards                | @VAR_BOARD_NAME_LOWER
 Example folder        | /examples/hsm_client/hsm_services/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

### Sample output

On successful boot, following message will be seen on terminal.

\cond SOC_AM263X

\snippet hsmclient.c hsm_services_example_qspi

\endcond

\cond SOC_AM263PX

\snippet hsmclient.c hsm_services_example_ospi
\endcond

\cond SOC_AM273X

\snippet hsmclient.c hsm_services_example_qspi_am273x

\endcond
