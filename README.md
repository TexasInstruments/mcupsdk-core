<div align="center">

<img src="https://upload.wikimedia.org/wikipedia/commons/b/ba/TexasInstruments-Logo.svg" width="150"><br/>
# MCU+ SDK

[Introduction](#introduction) | [Features](#features) | [Overview](#overview) | [Learn](#learn) | [Usage](#usage) | [Contribute](#contributing-to-the-project)

</div>

## Introduction

MCU+ SDK is a software development package designed for usage with Sitara MCU+ class of devices from Texas Instruments. These devices currently include

- [AM2431](https://www.ti.com/product/AM2431), [AM2432](https://www.ti.com/product/AM2432), [AM2434](https://www.ti.com/product/AM2434)
- [AM2634](https://www.ti.com/product/AM2634), [AM2634-Q1](https://www.ti.com/product/AM2634-Q1)
- [AM2732](https://www.ti.com/product/AM2732)
- [AM6411](https://www.ti.com/product/AM6411), [AM6412](https://www.ti.com/product/AM6412), [AM6421](https://www.ti.com/product/AM6421), [AM6422](https://www.ti.com/product/AM6422), [AM6441](https://www.ti.com/product/AM6441), [AM6442](https://www.ti.com/product/AM6442)

MCU+ SDK is designed with user experience and simplicity in mind. The SDK includes out-of-box application examples and peripheral usage examples to help users hit the ground running.

## Features

- Out of Box peripheral and application Examples
  - Peripheral Level Examples: UART, ADC, I2C, SPI etc.
  - Application Level Examples: Motor Drives, Industrial Communications etc.

- Protocol stacks and middleware
  - TinyUSB
  - LwIP
  - Various Industrial Protocol Stacks
  - FreeRTOS-FAT

- Drivers and Hardware Abstraction Layer
  - Board peripheral drivers - Flash, EEPROM, LED etc.
  - SoC peripheral drivers - I2C, SPI, OSPI, ADC etc.

- Industrial protocol firmware
  - ICCSG, ICSSM etc.

- OS kernel layer
  - Driver Porting Layer(DPL) which acts as an abstraction layer between driver and OS
  - Out of Box Support for
    - FreeRTOS
    - Baremetal i.e NO RTOS builds

## Overview

---

![Software Block Diagram](docs/sdk_block.png)

---

MCU+ SDK source comprises of multiple repositories with the current repository
at it's core. To build the SDK successfully, there are other repositories
that need to be cloned and are listed below:

- [Ethernet Low Level Driver (ENET LLD)](https://github.com/TexasInstruments/mcupsdk-enet-lld)
- [FreeRTOS-Kernel](https://github.com/TexasInstruments/mcupsdk-FreeRTOS-Kernel)
- [FreeRTOS-POSIX](https://github.com/TexasInstruments/mcupsdk-FreeRTOS-POSIX)
- [FreeRTOS-FAT](https://github.com/TexasInstruments/mcupsdk-FreeRTOS-FAT)

We use the [repo tool](https://gerrit.googlesource.com/git-repo) to clone and manage
multiple repositories. To setup the repo tool, navigate to [repo tool setup](#repo-tool-setup) section.

Prebuilt SDK installers  for specific devices are available at below links. Please note that installers are packaged specific to each device to reduce size.

- [AM243x MCU+ SDK](https://www.ti.com/tool/MCU-PLUS-SDK-AM243X)
- [AM263x MCU+ SDK](https://www.ti.com/tool/MCU-PLUS-SDK-AM263X)
- [AM273x MCU+ SDK](https://www.ti.com/tool/MCU-PLUS-SDK-AM273X)
- [AM64x  MCU+ SDK](https://www.ti.com/tool/download/MCU-PLUS-SDK-AM64X/08.04.00.17)

## Learn

TI has an amazing collection of tutorials on MCU+ Academy to help you get started.

- [AM243x MCU+ Academy](https://dev.ti.com/tirex/explore/node?node=A__AIo8NlrG3wSiDMUMT-uDWg__com.ti.MCU_PLUS_ACADEMY_AM243X__6zyEKJF__LATEST)

- [AM263x MCU+ Academy](https://dev.ti.com/tirex/explore/node?node=A__ADxJvKS8txu3Or8Qrf1ZiQ__com.ti.MCU_PLUS_ACADEMY_AM263X__rGFXMCu__LATEST)

- [AM273x MCU+ Academy](https://dev.ti.com/tirex/explore/node?node=A__AKYrWr.vaMFCFleLRjpj1g__com.ti.MCU_PLUS_ACADEMY_AM273X__VOimlOI__LATEST)

- [AM64x MCU+ Academy](https://dev.ti.com/tirex/explore/node?node=A__AO9O5HWh6-TdOdGD-bZe-g__com.ti.MCU_PLUS_ACADEMY_AM64X__n6QeJt5__LATEST)

## Usage

### Prerequisites

#### Supported HOST environments

- Windows 10 64bit
- Ubuntu 18.04 64bit

### Clone and build from GIT

#### Repo Tool Setup

MCU+ SDK has multiple components (in multiple repositories) and dependencies
(like compiler, CCS and other tools). We use repo tool from Google to manage these
multiple repositories. Currently there is no support for native windows shells like
CMD or Powershell. This will be added at a later point. Windows users can rely on
Git Bash for the repo setup. Follow the below mentioned steps to setup repo tool:

Make sure [python3 is installed](https://wiki.python.org/moin/BeginnersGuide/Download) and is in your OS path.

- Linux:
  Do the following in terminal
  ```bash
  curl https://storage.googleapis.com/git-repo-downloads/repo > ~/bin/repo
  chmod a+x ~/bin/repo
  echo "PATH=$HOME/bin:$PATH" >> ~/.bashrc
  source ~/.bashrc
  ```

- Git-Bash (Windows)
  Make sure that python 3 is callable as `python` from CMD. Do below in Git-Bash
  ```bash
  mkdir -p /c/ti
  curl https://storage.googleapis.com/git-repo-downloads/repo > /c/ti/repo
  echo "alias repo=\"winpty python /c/ti/repo\"" >> ~/.bashrc
  source ~/.bashrc
  ```

#### Cloning The Repositories

To clone the repositories using repo tool, do below in your workarea folder:

Note that depending on the SoC you're working with, the components you clone might be
slightly different. So please choose the manifest folder according to the SoC of your
interest. For example, we are showing for am263x below.

```bash
repo init -u https://github.com/TexasInstruments/mcupsdk-manifests.git -m am263x/dev.xml -b main
```

Note that repo uses symbolic links. So if you're on Windows and do not have permissions
to create symbolic links, the above command might fail for you. So you can either enable them
([refer this link](https://portal.perforce.com/s/article/3472)) or use the experimental
worktree feature of repo. To do this, initialize the repo like so:

```bash
repo init --worktree -u https://github.com/TexasInstruments/mcupsdk-manifests.git -m am263x/dev.xml -b main
```

After the repo is initialized, do a

```bash
repo sync
```

This should clone all the repositories required for MCU+ SDK development. Now download and install the dependencies.

#### Downloading And Installing Dependencies

Note that the dependencies are also soc specific, here we take an example of am263x.
You can replace that with the SoC of your choice like the `repo init` step.

**To download and install dependencies in linux, follow the below steps**:

Run the following from the same location where you have `mcu_plus_sdk` and `mcupsdk_setup`
folders.

```bash
./mcupsdk_setup/am263x/download_components.sh
```

This will install all the required dependencies including Code Composer Studio (CCS).
The script assumes that `mcu_plus_sdk` folder is in the same location from where
you have invoked the script, and that dependencies are installed into `${HOME}/ti`
location. If these defaults don't work for you, please pass these as arguments to
the script like

```bash
./mcupsdk_setup/am263x/download_components.sh --install_dir=/path/to/tools

OR

./mcupsdk_setup/am263x/download_components.sh --mcu_plus_sdk_folder=/path/to/mcu_plus_sdk/folder
```
and so on. For a complete list of arguments you can pass to the script, please run

```bash
./mcupsdk_setup/am263x/download_components.sh -h
```

**In windows the dependencies has to be manually installed. Given below are the steps**:

1. Download and install Code Composer Studio v12.1 from [here](https://www.ti.com/tool/download/CCSTUDIO "Code Composer Studio")
   - Install at default folder, C:\ti

2. Download and install SysConfig 1.14.0 from [here](https://www.ti.com/tool/download/SYSCONFIG/1.14.0.2667 "SYSCONFIG 1.14.0")
   - Install at default folder, C:/ti

3. Download and install GCC for Cortex A53 and ARM R5 from below link (only needed for AM64x developers)
   - [GNU-A](https://developer.arm.com/-/media/Files/downloads/gnu-a/9.2-2019.12/binrel/gcc-arm-9.2-2019.12-mingw-w64-i686-aarch64-none-elf.tar.xz)
   - [GNU-RM](https://developer.arm.com/-/media/Files/downloads/gnu-rm/7-2017q4/gcc-arm-none-eabi-7-2017-q4-major-win32.zip)
   - Install at default folder, C:/ti

4. Download and install Node.js v12.18.4 LTS
  - Go to the [NodeJS Website](https://nodejs.org/en/) and use the installer to
    download and install v12.18.4 of node. Install in the default directory.
  - After successful installation, run an `npm ci` inside the `mcu_plus_sdk` folder like so:
    ```bash
    $ cd mcu_plus_sdk/
    $ npm ci
    $ cd ../
    ```
    This should install the node packages required for the SDK.

1. Download and install doxygen,
   - Tested with 1.8.20
     - Download the correct version of doxygen for windows from [here](https://www.doxygen.nl/download.html)
     - Install and add the install path, typically, C:/Program Files/doxygen/bin to your windows PATH
   - Test by doing below on the command prompt
     ```
     $ doxygen -v
     1.8.20 (<commit SHA-ID>)
     ```

**Installing OpenSSL**

Some of the SDK signing scripts are dependent on OpenSSL v1.1.1. The v1.1.1 is
important, OpenSSL 3.0 is not compatible with the current signing scripts of SDK.

In Windows,
  - Download v1.1.1 from [here](https://slproweb.com/products/Win32OpenSSL.html)
  - You can install the "light" version which is smaller download size
  - Install to default path, which is C:/Program Files/OpenSSL-Win64/
  - When prompted select option to install binaries to /bin folder of installed path instead of Windows system path.
  - Add path to OpenSSL, to your environment "Path" variable in windows
    - `C:\Program Files\OpenSSL-Win64\bin`

In Linux,
  - There is a chance that OpenSSL is already installed. If not, here are the steps:
  - If you have Ubuntu 18.04, do below in Linux Ubuntu shell to install openssl
    -`$ sudo apt install openssl`

    If you have an Ubuntu version higher than that, make sure that you install the 1.1.1 version.
    You can get the 1.1.1 packages from [here](http://security.ubuntu.com/ubuntu/pool/main/o/openssl/).
    The packages required are openssl, libssl and libssl-dev

Test openssl version by running `openssl version` on a command prompt and make sure there is no error.
Example output is shown below,

```bash
  C:\> openssl version
  OpenSSL 1.1.1k  25 Mar 2021
```

Now that the dependencies are installed, you can start the repositories with a
default branch `dev` by doing below:

```bash
repo start dev --all
```

---

**NOTE**

- In Linux, you will need to run `$HOME/ti/ccs{version}/ccs/install_scripts/install_drivers.sh` script for setting COM
  port accesses correctly. Also add your user to groups `tty` and `dialout`. You can do

  ```
  sudo adduser $USER tty
  sudo adduser $USER dialout
  ```

- Please use the industrial communication libraries from the sdk installer due licensing restrictions.

---

### Building the SDK

#### Basic Building With Makefiles

---

**NOTE**

- Use `gmake` in windows, add path to gmake present in CCS at `C:\ti\ccsxxxx\ccs\utils\bin` to your windows PATH. We have
  used `make` in below instructions.
- Unless mentioned otherwise, all below commands are invoked from root folder of the "mcu_plus_sdk"  repository.
- Current supported device names are am64x, am243x, am263x, am273x and awr294x
- Pass one of these values to `"DEVICE="`
- You can also build components (examples, tests or libraries) in `release` or `debug`
  profiles. To do this pass one of these values to `"PROFILE="`

---

1. Run the following command to create makefiles, this step is optional since this is invoked as part of other steps as well,

   ```bash
   make gen-buildfiles DEVICE=am263x
   ```

2. To see all granular build options, run

   ```bash
   make -s help DEVICE=am263x
   ```
   This should show you commands to build specific libraries, examples or tests.

3. Make sure to build the libraries before attempting to build an example. For example,
   to build a Hello World example for AM263x, run the following:
   ```bash
   make -s -j4 libs DEVICE=am263x PROFILE=debug
   ```
   Once the library build is complete, to build the example run:
   ```bash
   make -s -C examples/hello_world/am263x-cc/r5fss0-0_nortos/ti-arm-clang all PROFILE=debug
   ```

4. Following are the commands to build **all libraries** and **all examples**. Valid PROFILE's are "release" or "debug"

   ```bash
   make -s -j4 clean DEVICE=am263x PROFILE=debug
   make -s -j4 all   DEVICE=am263x PROFILE=debug
   ```

### More information on SDK usage

For more details on SDK usage, please refer to the SDK userguide. User guides contain information on

- Building the SDK
- EVM setup,
- CCS Setup, loading and running examples
- Flashing the EVM
- SBL, ROV and much more.

Note that userguides are specific to a particular device. The links for all the supported devices are given below.

- [AM243x User Guide](https://software-dl.ti.com/mcu-plus-sdk/esd/AM243X/latest/exports/docs/api_guide_am243x/index.html)
- [AM263x User Guide](https://software-dl.ti.com/mcu-plus-sdk/esd/AM263X/latest/exports/docs/api_guide_am263x/index.html)
- [AM273x User Guide](https://software-dl.ti.com/mcu-plus-sdk/esd/AM273X/latest/exports/docs/api_guide_am273x/index.html)
- [AM64x  User Guide](https://software-dl.ti.com/mcu-plus-sdk/esd/AM64X/latest/exports/docs/api_guide_am64x/index.html)

The documentation can also be generated as mentioned in the below section.

### Generate Documentation

- Goto mcu_plus_sdk and type below to build the documentation for the device of interest

  ```bash
  make docs DEVICE=am263x
  ```

- Browse API guide by opening below file for a DEVICE of interest

  ```bash
  README_FIRST_*.html
  ```

- Also note that code snippets added to “docs_src\docs\api_guide\doxy_samples” is valid code that can compile.
  Above command also compiles the code snippets.

## Contributing to the project

This project is currently not accepting contributions. We expect to accept contributions from 3Q23.
