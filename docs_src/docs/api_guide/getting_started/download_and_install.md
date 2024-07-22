#  Download, Install and Setup SDK and Tools {#SDK_DOWNLOAD_PAGE}

[TOC]

\note The steps on this page need to be done once on a given host machine

## Host PC Requirements

To build applications using this SDK, one needs below host PC machine

- Windows PC
  - Windows 10 64bit
  - Minimum 4GB, >8GB RAM recommended
  - At least 10GB of hard disk space
- Linux PC
  - Ubuntu 18.04 64bit or higher
  - Minimum 4GB, >8GB RAM recommended
  - At least 10GB of hard disk space

## Download and Install the SDK

- Download the SDK installer and install at below path on your PC
  - Windows, C:/ti
  - Linux, ${HOME}/ti
- `${SDK_INSTALL_PATH}` in this user guide refers to the path, including the SDK folder name, where the SDK is installed.
  Example, in Windows, `${SDK_INSTALL_PATH}` will refer to the path `C:/ti/mcu_plus_sdk_{soc}_{version}`
\cond !(SOC_AM263PX || SOC_AM261X)
- You can also browse, download and install the SDK using TIREX as shown here, \ref TIREX_INTRO_PAGE.
\endcond

## Download and Install Additional SDK Tools

### SysConfig

- The SysConfig download home page is, https://www.ti.com/tool/SYSCONFIG
- Download SysConfig 1.20.0 and Install at below path,
  - Windows, C:/ti
  - Linux, ${HOME}/ti

\cond SOC_AM263X || SOC_AM263PX || SOC_AM273X || SOC_AM243X || SOC_AM261X
### Uniflash
- The TI Uniflash download home page is, https://www.ti.com/tool/UNIFLASH
- Download the latest Uniflash @VAR_UNIFLASH_VERSION and install at below path,
  - Windows, C:/ti
  - Linux, ${HOME}/ti
\endcond

\cond SOC_AM64X
### GCC AARCH64 Compiler
\attention GCC AARCH64 compiler installation is required only for A53 development in am64x

- Download GCC AARCH64 compiler @VAR_GCC_AARCH64_VERSION from the below link
  - Windows \htmllink{https://developer.arm.com/-/media/Files/downloads/gnu-a/9.2-2019.12/binrel/gcc-arm-9.2-2019.12-mingw-w64-i686-aarch64-none-elf.tar.xz , WINDOWS GCC AARCH64 CROSS COMPILER}
  - Linux \htmllink{https://developer.arm.com/-/media/Files/downloads/gnu-a/9.2-2019.12/binrel/gcc-arm-9.2-2019.12-x86_64-aarch64-none-elf.tar.xz, LINUX GCC AARCH64 CROSS COMPILER}
- Extract to below path,
  - Windows, C:/ti
  - Linux, ${HOME}/ti
\endcond

\cond SOC_AM64X
### GCC ARM (R5) Compiler
\attention GCC ARM compiler installation is required only for R5 GCC build

- Download GCC ARM compiler @VAR_GCC_ARM_VERSION from the below link
  - Windows \htmllink{https://developer.arm.com/-/media/Files/downloads/gnu-rm/7-2017q4/gcc-arm-none-eabi-7-2017-q4-major-win32.zip, WINDOWS GCC ARM CROSS COMPILER}
  - Linux \htmllink{https://developer.arm.com/-/media/Files/downloads/gnu-rm/7-2017q4/gcc-arm-none-eabi-7-2017-q4-major-linux.tar.bz2, LINUX GCC ARM CROSS COMPILER}
- Extract to below path,
  - Windows, C:/ti
  - Linux, ${HOME}/ti
\endcond

### Python3 {#INSTALL_PYTHON3}

\attention It is important to install Python 3.x. If you have Python 2.x installed, then additionally install
           Python 3.x and make sure the command python or python3 indeed points to Python 3.x

\attention All commands mentioned below should be typed in `cmd.exe` command console in Windows and `bash` terminal in Linux.

- Python scripts are used for below functionality in the SDK,
  - Flashing files to the flash on the EVM via UART.
  - Booting application on the EVM via UART
\cond SOC_AM64X || SOC_AM243X
  - SYSFW boardcfg formatting and C header file generation for SYSFW
\endcond
- Flashing files is the most popular reason why you would need python, so its strongly
  recommended to install it.
- In Windows,
  - Install python from, https://www.python.org/downloads/windows/
  - Confirm python is installed by typing below in a command prompt, make sure you see 3.x as the version

        C:\> python --version
        Python 3.9.1

  - If above command fails, then add path to Python to your environment "Path" variable, by default python is installed at below
    path

        C:\Users\{your username}\AppData\Local\Programs\Python\Python39

  - To add a new path to your environment variables, goto "Windows Task Bar Search"
    and search for "environment variables for your account"
    \imageStyle{ccs_setup_03.png,width:20%}
    \image html ccs_setup_03.png "Environment Variables For Your Account"

  - Click on "Path" variables, click on "Edit", click on "New"

  - Add the path to the folder where python in installed.

  - It is strongly recommended to move the path "up" in your path list by clicking the "Move Up" button until the path is at the top of the list.

  - Click "OK" to save the settings

  - Close your Windows command prompt and reopen it and then check if python is visible by doing below

        C:\> python --version
        Python 3.9.1

  - Check if the python package manager "pip" is installed, by default pip should be installed along with python.

        C:\> python -m pip --version
        pip 21.0.1 from C:\Users\{your username}\AppData\Local\Programs\Python\Python39\lib\site-packages\pip (python 3.9)

  - Install below additional packages via "pip" that are needed for the flashing tools. If you are behind a corporate firewall
    make sure to pass the server name and port for the proxy as shown below. If proxy is not needed keep `--proxy=` as blank.

        C:\> python -m pip install pyserial xmodem tqdm --proxy={your proxy server web-link and port}

- In Linux,
  - Do below in Linux bash shell to install python3

        $ sudo apt install python3 python3-pip

  - Check the python version by doing below

        $ python3 --version

  - Check if the python package manager "pip" is installed, by default pip should be installed along with python.

        $ pip3 --version

  - Install below additional packages via "pip" that are needed for the flashing tools. If you are behind a corporate firewall
    make sure to pass the server name and port for the proxy as shown below. If proxy is not needed keep `--proxy=` as blank.

        $ pip3 install pyserial xmodem tqdm --proxy={your proxy server web-link and port}

### OpenSSL {#INSTALL_OPENSSL}

- OpenSSL is needed for signing the bootloader and application images when booting using a bootloader.
- Current signing scripts of SDK support both v1.1.1 and v3 of OpenSSL.
- With am263x, am263px, am273x OpenSSL v3 is **recommended** as v1.1.1 has reached END OF LIFE.
- Download and install OpenSSL as below,
  - In windows,
    - Download OpenSSL v1.1.1 or v3 from https://slproweb.com/products/Win32OpenSSL.html
    - You can install the "light" version which is smaller download size
    - Install to default path, which is `C:/Program Files/OpenSSL-Win64/`
    - When prompted select option to install binaries to `/bin` folder of installed path instead of Windows system path.
    - Add path to OpenSSL, to your environment "Path" variable in windows

            C:/Program Files/OpenSSL-Win64/bin

    - In windows, there are multiple other options to install openssl as well, refer to this page if you want to install
      other versions of openssl, https://wiki.openssl.org/index.php/Binaries

  - In Linux,
    -  So if you have Ubuntu 22.04, do below in Linux Ubuntu shell to install openssl

            $ sudo apt install openssl

	- Make sure that you install the v1.1.1 or v3 of OpenSSL. You can get the v1.1.1 or v3 packages
    from http://security.ubuntu.com/ubuntu/pool/main/o/openssl/. The packages required are openssl, libssl and libssl-dev

- Test "openssl" by doing below on a command prompt and make sure there is no error. Example output on Windows is shown below,

        C:\> openssl version
        OpenSSL 1.1.1k  25 Mar 2021

    OR

        ~/workarea/mcu_plus_sdk$ openssl version
        OpenSSL 3.0.2 15 Mar 2022 (Library: OpenSSL 3.0.2 15 Mar 2022)

\cond SOC_AM64X || SOC_AM243X

### dfu-util {#INSTALL_DFU_UTIL}

\note USB2.0 DFU driver and application is tested with dfu-util verson `dfu-util 0.11-dev` and `dfu-util 0.8-dev`. It is recomended to use that same version as mentioned.

#### Windows

- Download the \htmllink{http://dfu-util.sourceforge.net/releases/dfu-util-0.8-binaries/win32-mingw32/dfu-util-static.exe, dfu-util} to your local system
- Rename it to `dfu-util.exe`
- Add the path to `dfu-util.exe` to the system environment variable `path`.
- check dfu-util version

            $ dfu-util --version

  \imageStyle{dfu_verison.png,width:60%}
  \image html dfu_version.png "dfu-util version"

##### Steps to install windows generic USB drivers.

- On windows we have to make sure that correct generic USB drivers are installed. We will use *Zadig* tool to install dependent USB libraries.
- Click here to Install [zadig](https://github.com/pbatard/libwdi/releases/download/v1.4.1/zadig-2.7.exe).

	- Connect EVM USB to PC and put the SOC in DFU boot mode. refer \ref EVM_SETUP_PAGE. Select **WinUSB** as shown in the figure below.


  \imageStyle{driver-install0.PNG,width:30%}
  \image html driver-install0.PNG "Step 1 select AM64x DFU from the list of devices."

	- Select 1.options 2. list all device. Find and select device named **AM64x DFU**. Click **Install WCID Driver**.

  \imageStyle{driver-install1.PNG,width:30%}
  \image html driver-install1.PNG "Step 2 Install drivers for AM64x DFU device."

	- Use following command to check whether ROM DFU device has been enumerated correctly or not.

			$ dfu-util -l

  \imageStyle{rom_dfu_enum.PNG,width:60%}
  \image html rom_dfu_enum.PNG "Step 2 Install drivers for AM64x DFU device."

##### Setps to Install drivers for using SBL DFU.

- After following above mentioned steps user will be able to perfom ROM DFU Boot. For using \ref EXAMPLES_DRIVERS_SBL_DFU_UNIFLASH DFU based flash writter tool follow below mentioned steps.

	- Follow the setps mentioned in the above section then use following command to boot the SBL DFU Uniflash flash writter.

			$ dfu-util -a 0 -i 0 -t 64 -D <path_to_sbl_dfu_uniflash.release.tiimage>

	- Once booted successfully, 1. open zadig tool 2. select options. 3. select list all devices. Select USB device named **AM64x-AM243x DFU**. Now select **Install WCID Driver** to install.

  \imageStyle{driver-install2.PNG,width:30%}
  \image html driver-install2.PNG "Install drivers for AM64x-AM243x DFU device."

Once the Drivers are installed successfully, user should be able to use the \ref TOOLS_FLASH_DFU_UNIFLASH tool to flash the application images.

#### Linux

- Download the  \htmllink{http://dfu-util.sourceforge.net/releases/dfu-util-0.8-binaries/win32-mingw32/dfu-util-static.exe, dfu-util} or use the package manager of your distribution to get the latest version.

            $ sudo apt install dfu-util

- check dfu-util version

            $ dfu-util --version
\endcond
\cond !SOC_AM65X
### PRU-CGT {#INSTALL_PRU-CGT}

\attention You MUST install PRU-CGT if trying to build PRU firmware.

\par
[PRU-CGT-2-3](https://www.ti.com/tool/PRU-CGT) (ti-pru-cgt) should be installed at: `C:/ti/` when working with PRU firmware development.

### Mono Runtime {#INSTALL_MONO}

\attention You MUST install mono runtime only if you're on Linux

- Mono runtime is required in Linux for creating bootloader images for application binaries.
- If you haven't installed mono, you are likely to run into this error while building:

        /bin/sh: 1: mono: not found

- Do below in Linux Ubuntu shell to install mono

        $ sudo apt install mono-runtime

\endcond
### Code Composer Studio (CCS)

To download, install and setup CCS, follow instructions on this page, \ref CCS_SETUP_PAGE .

## Download and Install Additional SDK Tools

### TI CLANG Compiler Toolchain {#INSTALL_TIARMCLANG}


\attention TI CLANG Compiler Toolchain is part of CCS, however if user wants to build example using makefile and CCS is not installed then this MUST be installed separately

- Download TI CLANG compiler toolchain @VAR_TI_ARM_CLANG_VERSION from below link
  - Windows, \htmllink{https://dr-download.ti.com/software-development/ide-configuration-compiler-or-debugger/MD-ayxs93eZNN/3.2.2.LTS/ti_cgt_armllvm_3.2.2.LTS_windows-x64_installer.exe, WINDOWS INSTALLER}
  - Linux, \htmllink{https://dr-download.ti.com/software-development/ide-configuration-compiler-or-debugger/MD-ayxs93eZNN/3.2.2.LTS/ti_cgt_armllvm_3.2.2.LTS_linux-x64_installer.bin, LINUX INSTALLER}
- Install at below path,
  - Windows, C:/ti
  - Linux, ${HOME}/ti
