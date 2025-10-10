#  Download, Install and Setup CCS {#CCS_SETUP_PAGE}

[TOC]

\note The steps on this page need to be done once on a given host machine
\cond SOC_AM263X || SOC_AM261X 
\note The screenshots shown are for AM263PX but can be similarly used for @VAR_SOC_NAME.
\endcond
## Download CCS

- Download CCS @VAR_CCS_VERSION, https://www.ti.com/tool/CCSTUDIO
- Unzip the file for Windows or Linux at any location on your host PC

## Install CCS

- Install CCS @VAR_CCS_VERSION by double clicking the installer file from the downloaded and un-zipped CCS package file.
\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
    \imageStyle{installer_file_am26.png,width:30%}
    \image html installer_file_am26.png "CCS Installer File"
\endcond

\cond SOC_AM273X || SOC_AM64X || SOC_AM243X
<br>
    \imageStyle{installer_file.png,width:30%}
    \image html installer_file.png "CCS Installer File"
\endcond


- Follow the steps and at below screen, recommend to keep install directory as default.
\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
    \imageStyle{install_directory_am26.png,width:40%}
    \image html install_directory_am26.png "CCS Install Path"
\endcond

\cond SOC_AM273X || SOC_AM64X || SOC_AM243X
<br>
    \imageStyle{install_directory.png,width:40%}
    \image html install_directory.png "CCS Install Path"
\endcond
    
\cond SOC_AM273X || SOC_AM64X || SOC_AM243X
- Follow the steps and at below screen, recommend to keep setup type as "custom"

    \imageStyle{setup_type.png,width:50%}
    \image html setup_type.png "CCS Setup Type"
\endcond
\cond SOC_AM243X || SOC_AM263X || SOC_AM263PX || SOC_AM273X || SOC_AM261X
- Follow the steps and at below screen, select the component as "AM2x Arm based high performance microcontrollers" to install @VAR_SOC_NAME related emulation and GELs

    \imageStyle{select_components_sitara_am26.png,width:50%}
    \image html select_components_sitara_am26.png "CCS Select Components"
\endcond
\cond SOC_AM64X || SOC_AM65X
- Follow the steps and at below screen, select the component as "Sitara AM3x, AM4x, AM5x and AM6x MPUs" to install @VAR_SOC_NAME related emulation and GELs

    \imageStyle{select_components_sitara_am6x.png,width:50%}
    \image html select_components_sitara_am6x.png "CCS Select Components"
\endcond
\cond SOC_AWR294X
- Follow the steps and at below screen, select the component as "Mmwave Sensors" to install @VAR_SOC_NAME related emulation and GELs

    \imageStyle{select_components_sitara_mmwave.png,width:25%}
    \image html select_components_sitara_mmwave.png "CCS Select Components"
\endcond
\cond SOC_AM62X
- Follow the steps and at below screen, select the component as "Sitara AM3x, AM4x, AM5x and AM6x MPUs" to install @VAR_SOC_NAME related emulation and GELs

    \imageStyle{select_components_sitara_am6x.png,width:50%}
    \image html select_components_sitara_am6x.png "CCS Select Components"
\endcond
- Follow the steps, until CCS is installed.

- If using Linux, additionally go through the instructions given at [CCS Linux Host Support](https://software-dl.ti.com/ccs/esd/documents/ccsv11_linux_host_support.html)
\cond SOC_AM273X || SOC_AM64X || SOC_AM243X
- Launch CCS and select the workspace.

    \imageStyle{first_launch.png,width:50%}
    \image html first_launch.png "CCS Select Workspace"
\endcond
\cond SOC_AM261X
## CSP Update for AM261x (CCS)

- Launch CCS. Go to Help ---> Check for Updates

    \imageStyle{ccs_update_check.png,width:20%}
    \image html ccs_update_check.png "Check for Updates Menu"

- The window will list the available updates. Select "AM2x Arm-based MCU Feature Support" and click next.

    \imageStyle{ccs_update_avail.png,width:40%}
    \image html ccs_update_avail.png "List of available updates"

- Accept the license agreement and click Finish.
- Restart CCS when prompted.

\endcond

## Check Packages as seen by CCS {#CCS_PACKAGE_CHECK}

- Launch CCS

\cond SOC_AM273X || SOC_AM64X || SOC_AM243X
- Goto "Window > Preferences"

    \imageStyle{ccs_setup_00.png,width:20%}
    \image html ccs_setup_00.png "CCS Preferences"

- Goto "Code Composer Studio > Products", make sure you see SysConfig @VAR_SYSCFG_VERSION listed here.
  - Sometimes, you need to click "Restore Defaults" and then "Refresh"

    \imageStyle{ccs_setup_01.png,width:50%}
    \image html ccs_setup_01.png "CCS Products"

- Goto "Code Composer Studio > Build > Compilers", make sure you see TI CLANG @VAR_TI_ARM_CLANG_VERSION listed here
  - Sometimes, you need to click "Restore Defaults" and then "Refresh"

    \imageStyle{ccs_setup_02.png,width:50%}
    \image html ccs_setup_02.png "CCS Compilers"
\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
- Goto "File > Preferences > Code Composer Studio Settings"

    \imageStyle{_ccs_setup_00_am26.png,width:50%}
    \image html ccs_setup_00_am26.png "CCS Preferences"

- Goto "Code Composer Studio > Products", make sure you see SysConfig @VAR_SYSCFG_VERSION listed here.

    \imageStyle{ccs_setup_01_am26.png,width:70%}
    \image html ccs_setup_01_am26.png "CCS Products"

- Goto "Code Composer Studio > Build > Compilers", make sure you see TI CLANG @VAR_TI_ARM_CLANG_VERSION listed here

    \imageStyle{ccs_setup_02_am26.png,width:70%}
    \image html ccs_setup_02_am26.png "CCS Compilers"
\endcond

\cond SOC_AM64X
## Create Target Configuration {#CCS_NEW_TARGET_CONFIG}
### AM64X-EVM
- Goto "View > Target Configuration"

    \imageStyle{new_target_config_00.png,width:20%}
    \image html new_target_config_00.png "Target Configuration Menu"

- Create a new target configuration

    \imageStyle{new_target_config_01.png,width:25%}
    \image html new_target_config_01.png "New Target Configuration"

- Give a nice name to the new target configuration, typically {soc name}_{JTAG type}

    \imageStyle{target_config_name.png,width:50%}
    \image html target_config_name.png "Target Configuration Name"

- Select connection as XDS110 USB Debug Probe

    \imageStyle{target_config_xds.png,width:50%}
    \image html target_config_xds.png "Select JTAG Connection"

- In "Board or Device" type "@VAR_SOC_NAME" and select "AM64x_GP_EVM"

    \imageStyle{ccs_target_config_00.png,width:50%}
    \image html ccs_target_config_00.png "Select @VAR_SOC_NAME EVM"

- Bypass not used CPUs: Go to "Advanced" tab and enable the "Bypass" option as shown in the below image. typically, ICSS_Gx are not used by most developers, so these can be bypassed. Note, you can always
  undo this change later, by editing the target configuration, should you need these CPUs.

    \imageStyle{ccs_target_config_01.png,width:50%}
    \image html ccs_target_config_01.png "Bypass unused targets"

- Click "Save" to save the newly created target configuration.

- The AM64x target configuration is just barebone, no GELs associated with PSC/PLL/DDR are loaded.
  AM64x_GP_EVM target configuration loads up and executes the appropriate GELs for the board.

- For SBL, you can use either, but for CCS load, you need to use AM64x_GP_EVM.

- Now you can move on to \ref EVM_SETUP_PAGE to prepare your EVM for running programs.

### AM64X-SK
- Goto "View > Target Configuration"

    \imageStyle{new_target_config_00.png,width:20%}
    \image html new_target_config_00.png "Target Configuration Menu"

- Create a new target configuration

    \imageStyle{new_target_config_01.png,width:25%}
    \image html new_target_config_01.png "New Target Configuration"

- Give a nice name to the new target configuration, typically {soc name}_{JTAG type}

    \imageStyle{target_config_name.png,width:50%}
    \image html target_config_name.png "Target Configuration Name"

- Select connection as XDS110 USB Debug Probe

    \imageStyle{target_config_xds.png,width:50%}
    \image html target_config_xds.png "Select JTAG Connection"

- In "Board or Device" type "@VAR_SOC_NAME" and select "AM64x_SK_EVM"

    \imageStyle{sk_ccs_target_config_00.png,width:50%}
    \image html sk_ccs_target_config_00.png "Select @VAR_SOC_NAME EVM"

- Bypass not used CPUs: Go to "Advanced" tab and enable the "Bypass" option as shown in the below image. typically, ICSS_Gx are not used by most developers, so these can be bypassed. Note, you can always
  undo this change later, by editing the target configuration, should you need these CPUs.

    \imageStyle{ccs_target_config_01.png,width:50%}
    \image html ccs_target_config_01.png "Bypass unused targets"

- Click "Save" to save the newly created target configuration.

- The AM64x-SK target configuration is just barebone, no GELs associated with PSC/PLL/DDR are loaded.
  AM64x_SK_EVM target configuration loads up and executes the appropriate GELs for the board.

- For SBL, you can use either, but for CCS load, you need to use AM64x_SK_EVM.

- Now you can move on to \ref EVM_SETUP_PAGE to prepare your EVM for running programs.
\endcond

\cond SOC_AM65X
## Create Target Configuration {#CCS_NEW_TARGET_CONFIG}
### AM65X-IDK
- Goto "View > Target Configuration"

    \imageStyle{new_target_config_00.png,width:20%}
    \image html new_target_config_00.png "Target Configuration Menu"

- Create a new target configuration

    \imageStyle{new_target_config_01.png,width:25%}
    \image html new_target_config_01.png "New Target Configuration"

- Give a nice name to the new target configuration, typically {soc name}_{JTAG type}

    \imageStyle{target_config_name.png,width:50%}
    \image html target_config_name.png "Target Configuration Name"

- Select connection as XDS110 USB Debug Probe

    \imageStyle{target_config_xds.png,width:50%}
    \image html target_config_xds.png "Select JTAG Connection"

- In "Board or Device" type "@VAR_SOC_NAME" and select "IDK_AM65x"

    \imageStyle{ccs_target_config_00.png,width:50%}
    \image html ccs_target_config_00.png "Select @VAR_SOC_NAME IDK"

- Bypass not used CPUs: Go to "Advanced" tab and enable the "Bypass" option as shown in the below image. typically, ICSS_Gx are not used by most developers, so these can be bypassed. Note, you can always
  undo this change later, by editing the target configuration, should you need these CPUs.

    \imageStyle{ccs_target_config_01.png,width:50%}
    \image html ccs_target_config_01.png "Bypass unused targets"

- Click "Save" to save the newly created target configuration.

- The AM65x target configuration is just barebone, no GELs associated with PSC/PLL/DDR are loaded.
  IDK_AM65X target configuration loads up and executes the appropriate GELs for the board.

- For SBL, you can use either, but for CCS load, you need to use IDK_AM65X.

- Now you can move on to \ref IDK_SETUP_PAGE to prepare your IDK for running programs.

\endcond

\cond SOC_AM243X
## Create Target Configuration {#CCS_NEW_TARGET_CONFIG}
### AM243X-LP
- Goto "View > Target Configuration"

    \imageStyle{new_target_config_00.png,width:20%}
    \image html new_target_config_00.png "Target Configuration Menu"

- Create a new target configuration

    \imageStyle{new_target_config_01.png,width:25%}
    \image html new_target_config_01.png "New Target Configuration"

- Give a nice name to the new target configuration, typically {soc name}_{JTAG type}

    \imageStyle{target_config_name.png,width:50%}
    \image html target_config_name.png "Target Configuration Name"

- Select connection as XDS110 USB Debug Probe

    \imageStyle{target_config_xds.png,width:50%}
    \image html target_config_xds.png "Select JTAG Connection"

- In "Board or Device" type "@VAR_SOC_NAME" and select "AM243x_LAUNCHPAD"

    \imageStyle{lp_ccs_target_config_00.png,width:50%}
    \image html lp_ccs_target_config_00.png "Select @VAR_SOC_NAME LP"

- Bypass not used CPUs: Go to "Advanced" tab and enable the "Bypass" option as shown in the below image. typically, ICSS_Gx are not used by most developers, so these can be bypassed. Note, you can always
  undo this change later, by editing the target configuration, should you need these CPUs.

    \imageStyle{ccs_target_config_01.png,width:50%}
    \image html ccs_target_config_01.png "Bypass unused targets"

- The AM2434_ALX target configuration is just barebone, no GELs associated with PSC/PLL/DDR are loaded.
  AM243x_LAUNCHPAD target configuration loads up and executes the appropriate GELs for the board.

- For SBL, you can use either, but for CCS load, you need to use AM243x_LAUNCHPAD.

- Click "Save" to save the newly created target configuration.

- Now you can move on to \ref EVM_SETUP_PAGE to prepare your AM243X-LP for running programs.

### AM243X-EVM
- Goto "View > Target Configuration"

    \imageStyle{new_target_config_00.png,width:20%}
    \image html new_target_config_00.png "Target Configuration Menu"

- Create a new target configuration

    \imageStyle{new_target_config_01.png,width:25%}
    \image html new_target_config_01.png "New Target Configuration"

- Give a nice name to the new target configuration, typically {soc name}_{JTAG type}

    \imageStyle{target_config_name.png,width:50%}
    \image html target_config_name.png "Target Configuration Name"

- Select connection as XDS110 USB Debug Probe

    \imageStyle{target_config_xds.png,width:50%}
    \image html target_config_xds.png "Select JTAG Connection"

- In "Board or Device" type "@VAR_SOC_NAME" and select "AM243x_GP_EVM"

    \imageStyle{ccs_target_config_00.png,width:50%}
    \image html ccs_target_config_00.png "Select @VAR_SOC_NAME EVM"

- Bypass not used CPUs, typically, ICSS_Gx are not used by most developers, so these can be bypassed. Note, you can always
  undo this change later, by editing the target configuration, should you need these CPUs.

    \imageStyle{ccs_target_config_01.png,width:50%}
    \image html ccs_target_config_01.png "Bypass unused targets"

- Click "Save" to save the newly created target configuration.

- The AM2434_ALV target configuration is just barebone, no GELs associated with PSC/PLL/DDR are loaded.
  AM243x_GP_EVM target configuration loads up and executes the appropriate GELs for the board.

- For SBL, you can use either, but for CCS load, you need to use AM243x_GP_EVM.

- Now you can move on to \ref EVM_SETUP_PAGE to prepare your EVM for running programs.

\endcond

\cond SOC_AM273X || SOC_AWR294X
## Create Target Configuration {#CCS_NEW_TARGET_CONFIG}
\cond SOC_AM273X
### AM273X-EVM
\endcond
\cond SOC_AWR294X
### AWR294X-EVM
\endcond

- Goto "View > Target Configuration"

    \imageStyle{new_target_config_00.png,width:20%}
    \image html new_target_config_00.png "Target Configuration Menu"

- Create a new target configuration

    \imageStyle{new_target_config_01.png,width:25%}
    \image html new_target_config_01.png "New Target Configuration"

- Give a nice name to the new target configuration, typically {soc name}_{JTAG type}

    \imageStyle{target_config_name.png,width:50%}
    \image html target_config_name.png "Target Configuration Name"

- Select connection as XDS110 USB Debug Probe

    \imageStyle{target_config_xds.png,width:50%}
    \image html target_config_xds.png "Select JTAG Connection"

- In "Board or Device" type "@VAR_SOC_NAME" and select "evmAM273x"

    \imageStyle{ccs_target_config_00.png,width:50%}
    \image html ccs_target_config_00.png "Select @VAR_SOC_NAME EVM"

- Click "Save" to save the newly created target configuration.

- Now you can move on to \ref EVM_SETUP_PAGE to prepare your EVM for running programs.

\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
## Create Target Configuration {#CCS_NEW_TARGET_CONFIG}
### @VAR_BOARD_NAME / @VAR_LP_BOARD_NAME

- Goto "Debug > Target Configurations"

    \imageStyle{new_target_config_00_am26.png,width:20%}
    \image html new_target_config_00_am26.png "Target Configuration Menu"

- Create a new target configuration using the + button

    \imageStyle{new_target_config_01_am26.png,width:25%}
    \image html new_target_config_01_am26.png "New Target Configuration"

- Give a nice name to the new target configuration, typically {soc name}_{JTAG type}

    \imageStyle{target_config_name.png,width:50%}
    \image html target_config_name.png "Target Configuration Name"

- Select connection as XDS110 USB Debug Probe

    \imageStyle{target_config_xds_am26.png,width:50%}
    \image html target_config_xds_am26.png "Select JTAG Connection"

\cond SOC_AM263X
- In "Board or Device" type "@VAR_SOC_NAME" and select "AM263x"
\endcond
\cond SOC_AM263PX 
- In "Board or Device" type "@VAR_SOC_NAME" and select "AM263Px"
\endcond
\cond SOC_AM261X
- In "Board or Device" type "@VAR_SOC_NAME" and select "AM261x"
\endcond
<br>
    \imageStyle{ccs_target_config_00.png,width:50%}
    \image html ccs_target_config_00.png "Select @VAR_SOC_NAME EVM"

- For CCs using ISO7221ADR (U53) for JTAG, lower the TCLK to 1MHz. Under Advanced tab, in the Connection Properties
    - Select 'The JTAG TCLK Frequency (MHz)' as "Fixed with user specified value"
    - Select '--Enter a value from 100.0kHz to 5.5MHz' as "1MHz"

    \imageStyle{ccs_target_config_freq.PNG,width:90%}
    \image html ccs_target_config_freq.PNG "Configuring JTAG TCLK frequency"

- Under Advanced tab – Select Cortex_R5_0
  - Make sure the device csp gel file path is in the Initialization script field

    \imageStyle{ccs_target_config_gel.png,width:90%}
    \image html ccs_target_config_gel.png "Initialization script"

- Click "Save" to save the newly created target configuration.

- Now you can move on to \ref EVM_SETUP_PAGE to prepare your EVM for running programs.

\endcond

\cond SOC_AM62X
## Create Target Configuration {#CCS_NEW_TARGET_CONFIG}

- Goto "View > Target Configuration"

    \imageStyle{new_target_config_00.png,width:20%}
    \image html new_target_config_00.png "Target Configuration Menu"

- Create a new target configuration

    \imageStyle{new_target_config_01.png,width:25%}
    \image html new_target_config_01.png "New Target Configuration"

- Give a nice name to the new target configuration, typically {soc name}_{JTAG type}

    \imageStyle{target_config_name.png,width:50%}
    \image html target_config_name.png "Target Configuration Name"

- Select connection as XDS110 USB Debug Probe

    \imageStyle{target_config_xds.png,width:50%}
    \image html target_config_xds.png "Select JTAG Connection"

- In "Board or Device" type "@VAR_SOC_NAME" and select "AM62x_SK_EVM"

    \imageStyle{ccs_target_config_00.png,width:50%}
    \image html ccs_target_config_00.png "Select @VAR_SOC_NAME EVM"

- Click "Save" to save the newly created target configuration.

- Now you can move on to \ref EVM_SETUP_PAGE to prepare your EVM for running programs.

\endcond


## Known Issues

### Using CCS debugger in Linux
- To use the debugger correctly with CCS in Linux, some additional steps are required. There is a message which is shown during the CCS installation to do this, but more often than not this is missed.
If you miss this, you might get an error similar to

\code
CS_DAP_0: Error initializing emulator: (Error -260 @ 0x0) An attempt to connect to the XDS110 failed. The cause may be one or more of: no XDS110 is connected,
invalid firmware update, invalid XDS110 serial number, or faulty USB cable. The firmware and serial number may be updated using the xdsdfu utility found in the
.../ccs_base/common/uscif/xds110 directory of your installation. View the XDS110SupportReadMe.pdf file there for instructions. (Emulation package 9.4.0.00129)
\endcode
\if SOC_AM65X
- There might be IDK specific issues in which the debugger maybe needs to be connected after the power is turned ON, or other similar issues. For this refer the IDK specific setup page at \ref IDK_SETUP_PAGE
\else
- There might be EVM specific issues in which the debugger maybe needs to be connected after the power is turned ON, or other similar issues. For this refer the evm specific setup page at \ref EVM_SETUP_PAGE
\endif

### Building projects in CCS in MAC machines {#CCS_MAC_ISSUE}

- While building imported projects in CCS in MAC machines you might face an error like this related to missing cryptography python module:

\code
from cryptography.hazmat.bindings._rust import openssl as rust_openssl
ImportError: dlopen(/Users/Library/Python/3.9/lib/python/site-packages/_cffi_backend.cpython-39-darwin.so, 0x0002): tried: '/Users/Library/Python/3.9/lib/python/site-packages/_cffi_backend.cpython-39-darwin.so' (mach-o file, but is an incompatible architecture (have 'arm64', need 'x86_64')), '/System/Volumes/Preboot/Cryptexes/OS/Users/Library/Python/3.9/lib/python/site-packages/_cffi_backend.cpython-39-darwin.so' (no such file), '/Users/Library/Python/3.9/lib/python/site-packages/_cffi_backend.cpython-39-darwin.so' (mach-o file, but is an incompatible architecture (have 'arm64', need 'x86_64'))
\endcode

#### Workaround 1 - Using python virtual environment

- Use the instruction on this page to [Create a Virtual Environment](https://packaging.python.org/en/latest/guides/installing-using-pip-and-virtual-environments) on your machine.
- After activating the virtual environment, pip install all required packages by typing 
    \code
        (.venv)$ python3 -m pip install -r {SDK_PATH}/requirements.txt
    \endcode
- Now launch CCS from this venv terminal by
    \code
        (.venv)$ open {PATH_TO_CCS_DIR}/ccs/eclipse/Ccsstudio.app
    \endcode
- Rebuilding the project should be successful now.

#### Workaround 2 - Modifying CCS Makefile

- Line `PYTHON=python3` in the `makefile_ccs_bootimage_gen` file of your project always defaults to the python present in your /usr/bin folder which might be an outdated python version incompatible with required packages.

- Update the python path in the file to point to your custom installed latest python. For example,
    \code
    ifeq ($(OS), Windows_NT)
        PYTHON=python
    else
        PYTHON=/opt/homebrew/bin/python3
    endif
    \endcode

- Rebuild the project.

#### Workaround 3 - Use CLI Makefile Build

- Build projects via make in command line instead of CCS. Refer to this section \ref MAKEFILE_BUILD_PAGE.