# SDL R5FCPU_UTILS Example {#EXAMPLES_SDL_R5F_CPU_STATIC_REGISTER}

[TOC]

# Introduction

This example demonstrates-
* How to read all the static registers of ARM Cortex R5F CPU, which all can be found in System control section in R5F TRM.

\cond SOC_AM263X || SOC_AM263PX
Use Cases
---------
* Read All the R5F CPU Static Registers.

\endcond


# Supported Combinations {#EXAMPLES_SDL_RESET_COMBOS}


\cond SOC_AM263X || SOC_AM263PX || SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/r5f_cpu_utils/

\endcond


# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref SDL_R5FCPU_PAGE

# Sample Output

\cond  SOC_AM263X || SOC_AM263PX || SOC_AM243X
Shown below is a sample output when the application is run for R5F,

For detail description of these register values, one can refer
[ARM CORTEX R5F TRM](https://developer.arm.com/documentation/ddi0460/c/System-Control/Register-descriptions?lang=en) in System control section.

\code

 ASM Test Application
All the register read are complete.
The MIDR register value  is  0x411fc153
The CTR register value  is  0x8003c003
The TCMTR register value  is  0x10001
The MPUIR register value  is  0x1000
The MPIDR register value  is  0xc0000000
The PFR0 register value  is  0x131
The PFR1 register value  is  0x1
The ID_DFR0 register value  is  0x10400
The ID_AFR0 register value  is  0x0
The ID_MMFR0 register value  is  0x210030
The ID_MMFR1 register value  is  0x0
The ID_MMFR2 register value  is  0x1200000
The ID_MMFR3 register value  is  0x211
The ID_ISAR0 register value  is  0x2101111
The ID_ISAR1 register value  is  0x13112111
The ID_ISAR2 register value  is  0x21232141
The ID_ISAR3 register value  is  0x1112131
The ID_ISAR4 register value  is  0x10142
The ID_ISAR5 register value  is  0x0
The CCSIDR register value  is  0xf00fe019
The CLIDR register value  is  0x9200003
The AIDR register value  is  0x0
The CSSELR register value  is  0x1
The SCTLR register value  is  0x1e5187d
The ACTLR register value  is  0x27
The SecondaryACTLR register value  is  0x400000
The CPACR register value  is  0xc0f00000
The MPURegionBaseADDR register value  is  0x70000000
The MPURegionEnableR register value  is  0x29
The MPURegionAccessControlR register value  is  0x20b
The RGNR register value  is  0x3
The BTCMRegionR register value  is  0x80019
The ATCMRegionR register value  is  0x25
The SlavePortControlR register value  is  0x0
The CONTEXTIDR register value  is  0x0
The ThreadProcessIDR1 register value  is  0x0
The ThreadProcessIDR2 register value  is  0x0
The ThreadProcessIDR3 register value  is  0x0
The nVALIRQSET register value  is  0x0
The nVALFIQSET register value  is  0x0
The nVALRESETSET register value  is  0x0
The nVALDEBUGSET register value  is  0x0
The nVALIRQCLEAR register value  is  0x0
The nVALFIQCLEAR register value  is  0x0
The nVALRESETCLEAR register value  is  0x0
The nVALDEBUGCLEAR register value  is  0x0
The BuildOption1R register value  is  0x80000
The BuildOption2R register value  is  0x8abfc5c7
The PinOptionR register value  is  0xc
The LLPPnormalAXIRR register value  is  0x0
The LLPPvirtualAXIRR register value  is  0x0
The AHBRR register value  is  0x5000004d
The CFLR register value  is  0x0
The PMOVSR register value  is  0x0
The DFSR register value  is  0x0
The ADFSR register value  is  0x0
The DFAR register value  is  0x0
The IFSR register value  is  0x0
The IFAR register value  is  0x0
The AIFSR register value  is  0x0
Test Name: ASM API TEST  PASSED

 All tests have passed



\endcode
\endcond
