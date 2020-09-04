# SafeRTOS {#KERNEL_SAFERTOS_PAGE}

[TOC]

## Introduction

The SDK will include only the safertos dpl layer. This needs the SafeRTOS kernal to build and use.
Please contact WITTENSTEIN for SafeRTOS kernal package.

## Extraction of safertos package

The safertos package needs to be extracted and placed in the below location

    ${SDK_INSTALL_PATH}/source/kernel/safertos/safeRTOS

After extraction, the safeRTOS should have the below folders structure. This is to make sure the safertos package extraction is proper for building with the SDK build system.

    | - safeRTOS
        | - api
        | - config
        | - kernel
        | - portable

## Building Safertos library {#KERNEL_SAFERTOS_LIB_BUILD}

The Safertos libs are not pre built when the SDK is installed.
You can use the makefiles to build please refer \ref MAKEFILE_BUILD_PAGE

The makefiles required to build safertos libs are part of the SDK package.

- To build the safertos library, do below,
    \code
    cd ${SDK_INSTALL_PATH}
    gmake -s -f makefile.awr294x safertos_c66.ti-c6000 PROFILE=release
    \endcode

- To clean  the safertos library, do below
    \code
    cd ${SDK_INSTALL_PATH}
    gmake -s -f makefile.awr294x safertos_c66.ti-c6000_clean PROFILE=release
    \endcode

## Features Supported

- C66 core support
- DPL interface for Task, semaphores, mutex, queues, timers
- C66 ISRs

## Migration of examples built with freertos to safertos

This section lists the changes required to build the example with safertos.
Assumption is application is using the DPL.
If it is calling any APIs of freertos directly they they need to be ported.

Refer the makefile and linker.cmd file from below path for building with safertos.
    \code
    ${SDK_INSTALL_PATH}\examples\hello_world_safertos\awr294x-evm\c66ss0_safertos\ti-c6000
    \endcode

Listed below are some major differences
- Update Include path
    - Remove the below freertos include path in makefile
    \code
	${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include
	${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_CGT/DSP_C66
	${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/awr294x/c66
    \endcode
    - Add the below safertos include path in makefile
    \code
	${MCU_PLUS_SDK_PATH}/source/kernel/safertos/safeRTOS/api/201_C66x
	${MCU_PLUS_SDK_PATH}/source/kernel/safertos/safeRTOS/api/PrivWrapperStd
	${MCU_PLUS_SDK_PATH}/source/kernel/safertos/safeRTOS/portable/201_C66x
	${MCU_PLUS_SDK_PATH}/source/kernel/safertos/safeRTOS/portable/201_C66x/005_TI_CGT
	${MCU_PLUS_SDK_PATH}/source/kernel/safertos/safeRTOS/kernel/include_api
	${MCU_PLUS_SDK_PATH}/source/kernel/safertos/safeRTOS/config
    \endcode

- Update libraries provided for Linker
    - Remove the freertos lib in makefile
    \code
    freertos.awr294x.c66.ti-c6000.${ConfigName}.lib
    \endcode
    - Add the safertos lib in makefile
    \code
    safertos.awr294x.c66.ti-c6000.${ConfigName}.lib
    \endcode

- Update the library path
    - Remove the lib path of freertos in makefile
    \code
    ${MCU_PLUS_SDK_PATH}/source/kernel/freertos/lib
    \endcode
    - Add the lib path of safertos in makefile
    \code
    ${MCU_PLUS_SDK_PATH}/source/kernel/safertos/lib
    \endcode

- Linker cmd file
    - Use the linker cmd file from the hello_world_safertos example.
    - Please do the changes as required for the application on top of this.

## See also

\ref KERNEL_DPL_PAGE
