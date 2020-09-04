# Using SDK with Makefiles {#MAKEFILE_BUILD_PAGE}

[TOC]

\note The steps on this page show screen shots and description based on Windows.
      However the steps in Linux would remain the same, unless mentioned otherwise.
\note In Linux, use `make` instead of `gmake`

\note The screen shots shown on this page are for AM64x MCU+ SDK v7.3.0. You would see
      your specific SOC and SDK version that is installed.
      However the instructions and steps mentioned remain the same, unless mentioned otherwise.

## Introduction

All SDK examples and libraries can be built using makefiles via command line. Using command line one can also
open SysConfig GUI to configure the example. This section provides basic instructions and tips on using makefiles.

## Enabling "make" in Windows

\attention This step needs to be done once on a Windows host machine. Nothing needs to be done for Linux.

- Windows, unlike Linux, does not come pre-installed with GNU make.
  CCS provides a GNU make executable. However the path to this GNU make executable should be
  visible to the windows command prompt.
- To add "make" to your path environment variable do below,

    \imageStyle{ccs_setup_03.png,width:20%}
    \image html ccs_setup_03.png "Add Environment Variable"

- Edit the "Path" variable and add the path to C:/ti/ccs@VAR_CCS_FOLDER_VERSION/ccs/utils/bin in the "Path" variable.
- Confirm that you are able to see gmake in Windows by doing below
    \code
    C:\ti>gmake -v
    \endcode

- The expected output is shown below
    \code
    GNU Make 4.1
    Built for Windows32
    Copyright (C) 1988-2014 Free Software Foundation, Inc.
    License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>
    This is free software: you are free to change and redistribute it.
    There is NO WARRANTY, to the extent permitted by law.
    \endcode

## Building Libraries with Makefiles

\note CCS projects are not supported for libraries and if any library source file or header file
      is modified the libraries need to be rebuilt as shown below

- When SDK is installed the drivers, FreeRTOS, NORTOS and other libraries are pre-built for
  both "debug" mode and "release" mode. However you can modify the library source code
  and rebuild all the libraries using makefiles.

- To rebuild the libraries, do below,
    \code
    cd ${SDK_INSTALL_PATH}
    gmake -s libs PROFILE=release
    \endcode

- To clean all library generated object files and libraries, do below
    \code
    cd ${SDK_INSTALL_PATH}
    gmake -s  libs-clean PROFILE=release
    \endcode

- Additionally, you can pass `PROFILE=debug` to build the libraries without optimizations. This
  helps when debugging code in CCS debugger.

## Building examples with makefiles {#MAKEFILE_EXAMPLE_BUILD}

- You can build example applications using CCS projects as well as makefiles.
- To build using makefiles, firstly identify the example to build by doing below
    \code
    gmake -s help
    \endcode

- This will list all the make "commands" that you can execute. A sample output snippet is shown below,
    \code
    \# Notes,
    \# - Use -j to invoke parallel builds
    \# - Use PROFILE=debug or PROFILE=release [default] to build in debug or release profile
    \#
    \# Overall build targets,
    \# ======================
    \# gmake -s -f makefile.{soc} help

    ...
    \# Example build targets,
    \# ======================
    \# gmake -s -C examples/hello_world/{board}/r5fss0-0_nortos/ti-arm-clang [all clean syscfg-gui syscfg]
    \# gmake -s -C examples/hello_world/{board}/r5fss0-0_freertos/ti-arm-clang [all clean syscfg-gui syscfg]
    \# gmake -s -C examples/hello_world/{board}/m4fss0-0_nortos/ti-arm-clang [all clean syscfg-gui syscfg]
    \# gmake -s -C examples/hello_world/{board}/m4fss0-0_freertos/ti-arm-clang [all clean syscfg-gui syscfg]
    ...
    \# System Example build targets,
    \# =============================
    \# gmake -s -C examples/drivers/ipc/ipc_notify_echo/{board}/system_freertos_nortos [all clean syscfg-gui syscfg]
    \# gmake -s -C examples/drivers/ipc/ipc_rpmsg_echo/{board}/system_freertos_nortos [all clean syscfg-gui syscfg]
    ...
    \endcode

- All examples are arranged as below
    \code
    examples/{component or module}/{optional sub-module or sub-component}/
                   |
                   + -- {example name}/{board on which this example can run}/
                                         |
                                         + -- {cpu}_{os}/{compiler toolchain}
    \endcode

- To build a given example pick the "example combo" that you want to build and copy-paste that command on the command prompt as below
    \code
    cd ${SDK_INSTALL_PATH}
    gmake -s -C examples/hello_world/{board}/m4fss0-0_freertos/ti-arm-clang all PROFILE=release
    \endcode

- As shown above, one can also pass `PROFILE=release` or `PROFILE=debug` to build in "release" or "debug" profile.

- Instead of "all" you can also pass other arguments as below.

- To clean a example do,
    \code
    cd ${SDK_INSTALL_PATH}
    gmake -s -C examples/hello_world/{board}/r5fss0-0_freertos/ti-arm-clang clean PROFILE=release
    \endcode

- To launch SysConfig GUI from command line for a example do, (see also \ref SYSCONFIG_INTRO_PAGE)
    \code
    cd ${SDK_INSTALL_PATH}
    gmake -s -C examples/hello_world/{board}/r5fss0-0_freertos/ti-arm-clang syscfg-gui
    \endcode

- To simply build all examples, do below
    \code
    cd ${SDK_INSTALL_PATH}
    gmake -s  examples PROFILE=release
    \endcode

- Explore additional commands listed via `gmake -s help` for more options

## Building System Examples with Makefiles

- Some examples, to run correctly, need multiple binaries on different CPUs to be build, loaded and run.
  Such examples are called "system" examples. One such example is inter-processor communication example,
  which shows multiple CPUs communicating among each other.

- These examples are listed under `System Example build targets` when `gmake -s help` is done
    \code
    ...
    \# System Example build targets,
    \# =============================
    \# gmake -s -C examples/drivers/ipc/ipc_notify_echo/{board}/system_freertos_nortos [all clean syscfg-gui syscfg]
    \# gmake -s -C examples/drivers/ipc/ipc_rpmsg_echo/{board}/system_freertos_nortos [all clean syscfg-gui syscfg]
    ...
    \endcode

- When the command shown under "System Example ..." is executed, multiple makefiles are executed to build executables for all CPUs associated with that "system" example

- You can also launch SysConfig using the system example makefile as shown in below example, (see also \ref SYSCONFIG_INTRO_PAGE)
    \code
    gmake -s -C examples/drivers/ipc/ipc_notify_echo/{board}/system_freertos_nortos syscfg-gui
    \endcode

- Here SysConfig will show the configuration of all CPUs in a single SysConfig window.

    \imageStyle{syscfg_00.png,width:70%}
    \image html syscfg_00.png "SysConfig Multi-CPU View"

## Building the Whole SDK with Makefiles

- The whole SDK can be built using makefiles by running below command,
    \code
    cd ${SDK_INSTALL_PATH}
    gmake -s  all PROFILE=release
    \endcode

- Similarly to clean the whole SDK, do below
    \code
    cd ${SDK_INSTALL_PATH}
    gmake -s  clean PROFILE=release
    \endcode

## Load and Run Executables Built with Makefiles

- The generated binary from makefile is located as the below path
    \code
    examples/{component or module}/{optional sub-module or sub-component}/
                   |
                   + -- {example name}/{board on which this example can run}/
                                         |
                                         + -- {cpu}_{os}/{compiler toolchain}
                                                            |
                                                            + -- {example_name}.release.out
                                                            + -- {example_name}.debug.out
    \endcode

- Simply load and run binary .out on EVM using CCS (see \ref CCS_LAUNCH_PAGE)

- The same folder also has .map files which have more information about the binary size and memory section
  used, and so on.

- In case of "system" examples, one need to load the binaries for each CPU one by one in CCS and then run each CPU.
  Typically the order of load and run of different CPUs is not important.

## Tips and Tricks when working with Makefiles

- In general, the makefiles are written to be simple and use very basic GNU make syntax and options.
  You can simply open the makefile located within
  a example or library folder to see the different compile options, include paths, files, include directory,
  pre-processor defines etc that are used to build that example or library.

- The only common file included by all makefiles is the top level `${SDK_INSTALL_PATH}/imports.mak`.
  This file defines paths to common tools
  like compiler, syscfg, CCS that are used by all examples. If you have installed the tools like
  CCS, SysConfig, Compiler at non-default paths, then modify the paths defined in this file.

- When a specific example is built the libraries used by it are not checked for changes i.e. dependency is not set.
  So make sure to do a `gmake -s  libs PROFILE={debug or release}` to build all libraries in case you have changed any library source or header file.
  This checks and triggers a "incremental" build across all libraries.

- On Linux, you can pass `-j` option to allow make to use all CPUs on your host machine when running make.
    This will make the build time significantly faster depending on your host machine speed.

- On Windows, the windows command prompt sometimes locks up when make is invoked with `-j` option.
  One solution is use a different shell in windows like the bash shell provided by GIT for windows, https://git-scm.com/download/win .
  With the bash shell for windows, you can pass `-j` to make the builds fast

- You can pass `-s` to build in "silent" mode. This will suppress the actual command that is executed. So if
  you dont pass `-s` then the console will show the exact command that is executed by make. This can be useful in debugging
  error, if any.

- You can pass `PROFILE=debug` or `PROFILE=release` as arguments to build in debug mode or release mode

- You can also build a example by going to the example folder where the makefile is located and simply type one of below,
    \code
    cd ${SDK_INSTALL_PATH}/examples/hello_world/{board}/r5fss0-0_freertos/ti-arm-clang
    gmake -s all PROFILE={debug or release}   # Build the example
    gmake -s clean PROFILE={debug or release} # Clean the example
    gmake -s syscfg-gui                       # Launch SysConfig GUI for the example
    \endcode

- In general, explore the commands and options listed under `gmake -s help`
