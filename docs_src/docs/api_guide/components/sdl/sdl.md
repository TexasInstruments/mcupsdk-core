# Software Diagnostics Library (SDL) {#SDL_PAGE}

[TOC]

# Introduction

The @VAR_SOC_NAME family of SoCs provides various safety mechanisms and features, as well as recommendations for usage of these safety mechanisms and features. The Software Diagnostic Library (SDL) provides interfaces to these safety mechanisms and features. The provides these interfaces to assist in the development of software applications involving Functional Safety.

In an application involving functional safety, the detection of random hardware faults and ability to take the appropriate response to get the system to a safe state is of utmost importance. Methods to detect and respond to faults in a system are called functional safety mechanisms or safety functions. Examples of safety mechanisms available on an SoC include error correction/detection (ECC) on memory regions, Error Signaling Module (ESM) to monitor error events, etc.

The safety-critical processor product family provides various hardware functional safety mechanisms. For example, this software release provides an API to configure ECC and a reference example to set up interrupts to check on ECC error events detected by hardware. Overall, the system integrator can use this API and implement software diagnostics to meet the safety system goals.

The user of this document should have a general familiarity with the safety-critical processor family.

The Software Diagnostics Library consists of different blocks for Error Capture and Safety Mechanisms. Error response is managed by the Application based on the device Safety Manual requirements. The interface for the Application is in the form of software APIs. The following diagram shows the high-level blocks of the SDL as well as the overall system. The application may use either no OS or an OS. In the diagram an OS is shown as an example only. This is an overview and does not list all the IPs supported as part of the SDL.

In the following diagram, the green blocks represent the scope of the SDL. The dark blue is the application, and the light blue represents external modules used by the application along with SDL.

\imageStyle{SDL_Arch.png,width:60%}
\image html SDL_Arch.png "SDL Architecture"

The Software Diagnostics Library provides the functionality for implementing hardware safety mechanisms that can be run during the various operation modes of the device. The functions of the SDL which are used by the application during the various modes are as follows:

- Startup of device
	- Self-test diagnostics to verify correct operation of the device
	- SDL APIs to initialize continuous diagnostics

- Runtime
	- Notification of errors detected by the continuous diagnostics through the ESM handler
	- SDL APIs to execute periodic diagnostics

\cond SOC_AM273X
**Note:** A document to show the SDL API mapping of Sitara MCU Recommended Safety Functions exists. Please contact FAE to get it.
\endcond
SDL consists of below sub-modules

\cond SOC_AM64X || SOC_AM243X
- \subpage SDL_MCRC_PAGE
- \subpage SDL_DCC_PAGE
- \subpage SDL_DPL_PAGE
- \subpage SDL_ESM_PAGE
- \subpage SDL_STOG_PAGE
- \subpage SDL_VTM_PAGE
- \subpage SDL_POK_PAGE
- \subpage SDL_MTOG_PAGE
- \subpage SDL_PBIST_PAGE
- \subpage SDL_ECC_PAGE
- \subpage SDL_RTI_PAGE
- \subpage SDL_ROM_CHECKSUM_PAGE
- \subpage SDL_LBIST_PAGE
\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AWR294X || SOC_AM261X
- \subpage SDL_DPL_PAGE
- \subpage SDL_ESM_PAGE
- \subpage SDL_DCC_PAGE
- \subpage SDL_RTI_PAGE
- \subpage SDL_MCRC_PAGE
- \subpage SDL_ECC_PAGE
- \subpage SDL_PBIST_PAGE
- \subpage SDL_CCM_PAGE
- \subpage SDL_STC_PAGE
- \subpage SDL_ECC_BUS_SAFETY_PAGE
\endcond

\cond  SOC_AM273X
- \subpage SDL_DPL_PAGE
- \subpage SDL_ESM_PAGE
- \subpage SDL_RTI_PAGE
- \subpage SDL_MCRC_PAGE
- \subpage SDL_ECC_PAGE
- \subpage SDL_PBIST_PAGE
- \subpage SDL_CCM_PAGE
- \subpage SDL_STC_PAGE
- \subpage SDL_ECC_BUS_SAFETY_PAGE
\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
- \subpage SDL_R5FCPU_PAGE
\endcond
\cond SOC_AM263PX
- \subpage SDL_TMU_ROM_CHECKSUM_PAGE
- \subpage SDL_STOG_PAGE
- \subpage SDL_VTM_PAGE
\endcond
\cond SOC_AM261X
- \subpage SDL_TMU_ROM_CHECKSUM_PAGE
- \subpage SDL_STOG_PAGE
- \subpage SDL_VTM_PAGE
\endcond

\cond SOC_AM243X
- \subpage SDL_R5FCPU_PAGE
\endcond

\cond SOC_AM273X || SOC_AWR294X
- \subpage SDL_HWA_PAGE
- \subpage SDL_RESET_PAGE
\endcond

