# SDL CCM {#EXAMPLES_SDL_CCM}

[TOC]

# Introduction

This example demonstrates the usage of the SDL R5 CCM module. The example shows how to setup and use the R5 CCM Safety Diagnostic operation.

Use Cases
---------

 Use Case | Description
 ---------|------------
 UC-1     | self test on R5 CCM.
 UC-2     | self test with error force on R5 CCM.


# Supported Combinations {#EXAMPLES_SDL_CCM_COMBOS}

\cond SOC_AM263X || SOC_AM273X || SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/ccm/sdl_ccm_example/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# Steps to Test the Example

- CCM example should only be tested in QSPI boot mode and should not be tested in no boot mode with debugger connected.
- Follow the below procedure to test the CCM and CCM_VIM example.
  1. A quick recap of steps done so far that are needed for the flashing to work
     - Make sure the UART port used for terminal is identified as mentioned in \ref CCS_UART_TERMINAL
     - Make sure python3 is installed as mentioned in \ref INSTALL_PYTHON3
     - Make sure you have the EVM power cable and UART cable connected as shown in \ref EVM_CABLES
  2. Build the hello world application as mentioned in Build a Hello World example, \ref GETTING_STARTED_BUILD
  3. As part of the build process in the final step a file with extension .appimage is generated. This is the file we need to flash.
  4. When building with makefiles and single-core projects, this file can be found here

			${SDK_INSTALL_PATH}/examples/sdl/ccm/sdl_ccm_example/{board}/r5fss0-0_nortos/ti-arm-clang/sdl_ccm_example.release.appimage

  5. A default configuration file can be found at below path. You can edit this file directly or take a copy and edit this file.

			${SDK_INSTALL_PATH}/tools/boot/sbl_prebuilt/{board}/default_sbl_qspi.cfg

  6. Edit below line in the config file to point to your application .appimage file. Give the absolute path to the .appimage file 
    or path relative to ${SDK_INSTALL_PATH}/tools/boot. **Make sure to use forward slash / in the filename path**.
		
			--file=../../examples/sdl/ccm/sdl_ccm_example/{board}/r5fss0-0_nortos/ti-arm-clang/sdl_ccm_example.release.appimage --operation=flash --flash-offset=0x80000
    
	- **NOTE** : edit sdl_ccm_example to sdl_ccm_vim for ccm vim example testing.
  7. Save and close the config file.
  8. Flash the application by follow the steps mentioned in the page, \ref GETTING_STARTED_FLASH
  
# See Also

\ref SDL_CCM_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\cond (SOC_AM263X) || (SOC_AM273X) || (SOC_AWR294X)
\code
R5F0 CORE:

R5 CPU Application

R5F-CPU example application CCM_Test_init: Init MCU ESM complete

CCM_Test_init: CCM Init complete

R5F-CPU example application CCM_Test_init: CCM Init complete

CCM Functional Test

R5F-CPU example application CCM self test: starting

R5F-CPU example application CCM Self Test complete

R5F-CPU example application CCM self test with error forcing: starting

R5F-CPU example application CCM Self Test with error forcing complete

R5F-CPU example application CCM inject  error: test starting

R5F-CPU example application CCM inject Test complete

CPU Functionality Passed.

All tests have passed.

\endcode
\endcond

\cond (SOC_AM263X)
\code
R5F1 CORE:

R5 CPU Application

R5F-CPU example application CCM_Test_init: Init MCU ESM complete

CCM_Test_init: CCM Init complete

R5F-CPU example application CCM_Test_init: CCM Init complete

CCM Functional Test

R5F-CPU example application CCM self test: starting

R5F-CPU example application CCM Self Test complete

R5F-CPU example application CCM self test with error forcing: starting

R5F-CPU example application CCM Self Test with error forcing complete

R5F-CPU example application CCM inject  error: test starting

R5F-CPU example application CCM inject Test complete

CPU Functionality Passed.

All tests have passed.
\endcode
\endcond