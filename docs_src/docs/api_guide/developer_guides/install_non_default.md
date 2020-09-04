# Installing SDK at non-default location {#INSTALL_NON_DEFAULT}

[TOC]

\note The screen shots shown on this page are for AM64x MCU+ SDK v7.3.0. You would see
      your specific SOC and SDK version that is installed.
      However the instructions and steps mentioned remain the same, unless mentioned otherwise.

## Introduction

The user guide in general recommends to install the SDK and tools at default paths for simplicity.
However the SDK and tools can be installed at non-default locations as well. This section lists some details
you need to follow when installing the SDK and tools at non-default locations.

## SDK Installation

### Using Makefiles

- You can install SDK anywhere and there is nothing more to do.

- Each makefile has a relative path to the root of the SDK. You can install the SDK
  anywhere and the makefiles inside the SDK will build the libraries and example correctly

### Using CCS Projects

- After installing the SDK at non-default location, you need to
  make the sure the SDK is visible to CCS IDE by following the steps shown here \ref CCS_PACKAGE_CHECK

- If you dont see the SDK here, then click "Add" in the "Products" tab to add path to your SDK install location. And then click "Refresh" to make sure the SDK is now seen by CCS.

    \imageStyle{ccs_setup_01.png,width:50%}
    \image html ccs_setup_01.png "CCS Products"

## SysConfig Installation

### Using Makefiles

- Edit the below file to point to SysConfig install location

        ${SDK_INSTALL_PATH}/imports.mak

- Edit the below variable in this file, depending on Linux or Windows, to point to the SysConfig install location. Give absolute path with forward slash `/`.

        SYSCFG_PATH ?= $(TOOLS_PATH)/sysconfig_@VAR_SYSCFG_VERSION

### Using CCS Projects

- After installing SysConfig at non-default location, you need to
  make the sure it is visible to CCS IDE by following the steps shown here \ref CCS_PACKAGE_CHECK

- If you don't see SysConfig here, then click "Add" in the "Products" tab to add path to your install location. And then click "Refresh"
  to make sure SysConfig is now seen by CCS.

    \imageStyle{ccs_setup_01.png,width:50%}
    \image html ccs_setup_01.png "CCS Products"

## TI CLANG Compiler Toolchain Installation

### Using Makefiles

- Edit the below file to point to compiler install location

        ${SDK_INSTALL_PATH}/imports.mak

- Edit the below variable in this file, depending on Linux or Windows, to point to the compiler install location. Give absolute path with forward slash `/`.

        CGT_TI_ARM_CLANG_PATH=$(TOOLS_PATH)/ti-cgt-armllvm_1.3.1.LTS

### Using CCS Projects

- After installing the compiler at non-default location, you need to
  make the sure it is visible to CCS IDE by following the steps shown here \ref CCS_PACKAGE_CHECK

- If you don't see the compiler here, then click "Add" in the "Compilers" tab to add path to your install location. And then click "Refresh"
  to make sure the compiler is now seen by CCS.

    \imageStyle{ccs_setup_02.png,width:50%}
    \image html ccs_setup_02.png "CCS Compilers"

## CCS Installation

### Using Makefiles

- Edit the below file to point to CCS install location

        ${SDK_INSTALL_PATH}/imports.mak

- Edit the below variable in this file, depending on Linux or Windows, to point to CCS install location. Give absolute path with forward slash `/`.

        CCS_PATH?=$(TOOLS_PATH)/ccs@VAR_CCS_FOLDER_VERSION/ccs

### Using CCS Projects

- After installing CCS at non-default location, make the sure the SDK, SysConfig and compiler is visible to CCS IDE, by following the steps shown here \ref CCS_PACKAGE_CHECK

- If you don't see either the SDK or tools here, then click "Add" in the "Products" or "Compilers" tab to add path to your install location. And then click "Refresh" to make sure the packages are now seen by CCS.

    \imageStyle{ccs_setup_01.png,width:50%}
    \image html ccs_setup_01.png "CCS Products"

    \imageStyle{ccs_setup_02.png,width:50%}
    \image html ccs_setup_02.png "CCS Compilers"

## Other Tools

The SDK needs other tools like openssl, python 3.x. These can be installed anywhere, just make sure the tools
are visible to the command prompt in Windows or Linux. See \ref SDK_DOWNLOAD_PAGE for more information on how to
check if the tools are visible and installed as expected.
