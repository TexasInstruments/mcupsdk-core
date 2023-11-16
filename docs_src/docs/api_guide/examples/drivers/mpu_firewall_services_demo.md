#  MPU Firewall Service DEMO {#EXAMPLES_MPU_FIREWALL_SERVICES_DEMO}

[TOC]

# Introduction

This example demonstrates the use of HSM Firewall services.

2 services are available.
1. Set Firewall Service is used to set firewall configuration.
2. Set Firewall Interrupt service is responsible for the 4 operations mentioned below:
    * 2.1\. Interrupt enable for protection and address violation
    * 2.2\. Clearing Interrupt enable.
    * 2.3\. Clearing Interrupt status.
    * 2.4\. Clearing the current fault so that another can be captured.

# Supported Combinations {#EXAMPLES_MPU_FIREWALL_SERVICES_DEMO_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

 Parameter             | Value
 ----------------------|-----------
 CPU + OS              | r5fss0_0 nortos
 ^                     | r5fss0_1 nortos
 Toolchain             | ti-arm-clang
 Boards                | @VAR_BOARD_NAME_LOWER
 Example folder        | /examples/hsm_client/mpu_firewall_services_demo/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

### Sample Output

On successful boot, following message will be seen on terminal. Please note that
the R5F application has not been side-loaded yet.

\cond SOC_AM263X

\snippet hsmclient.c hsm_services_example_qspi

\endcond

\cond SOC_AM263PX

\snippet hsmclient.c hsm_services_example_ospi

\endcond

