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

R5F CPU STATIC REGISTER READ Start... 

The MPU Register read started for MPU REGION -> 0x0 

The SCTLR register value  is  0x1e5187d 
The MPUIR register value  is  0x1000 
The RGNR register value  is  0x0 
The MPURbaseAddr register value  is  0x0 
The MPURsize register value  is  0x3d 
The MPURaccessControl register value  is  0x1204 

The MPU Register read started for MPU REGION -> 0x1 

The SCTLR register value  is  0x1e5187d 
The MPUIR register value  is  0x1000 
The RGNR register value  is  0x1 
The MPURbaseAddr register value  is  0x0 
The MPURsize register value  is  0x1d 
The MPURaccessControl register value  is  0x20b 

The MPU Register read started for MPU REGION -> 0x2 

The SCTLR register value  is  0x1e5187d 
The MPUIR register value  is  0x1000 
The RGNR register value  is  0x2 
The MPURbaseAddr register value  is  0x80000 
The MPURsize register value  is  0x1d 
The MPURaccessControl register value  is  0x20b 

The MPU Register read started for MPU REGION -> 0x3 

The SCTLR register value  is  0x1e5187d 
The MPUIR register value  is  0x1000 
The RGNR register value  is  0x3 
The MPURbaseAddr register value  is  0x70000000 
The MPURsize register value  is  0x29 
The MPURaccessControl register value  is  0x20b 

All the R5F MPU Static register read are complete. 

The SCTLR register value  is  0x1e5187d 
The ACTLR register value  is  0x2f 
The SecondaryACTLR register value  is  0x400000 
The CPACR register value  is  0xc0f00000 
The BTCMRegionR register value  is  0x80019 
The ATCMRegionR register value  is  0x25 
The SlavePortControlR register value  is  0x0 
The CONTEXTIDR register value  is  0x0 
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
The PMCNTENSET register value  is  0x80000000 
The PMCR register value  is  0x41151801 
The PMUSERENR register value  is  0x0 
The PMINTENSET register value  is  0x0 
The PMINTENCLR register value  is  0x0 

R5F CPU STATIC REGISTER READ Complete! 

All test have passed. 

\endcode
\endcond
