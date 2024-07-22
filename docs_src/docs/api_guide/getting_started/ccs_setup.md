#  Download, Install and Setup CCS {#CCS_SETUP_PAGE}

[TOC]

\note The steps on this page need to be done once on a given host machine

## Download CCS

- Download CCS @VAR_CCS_VERSION, https://www.ti.com/tool/CCSTUDIO
- Unzip the file for Windows or Linux at any location on your host PC

## Install CCS

- Install CCS @VAR_CCS_VERSION by double clicking the installer file from the downloaded and un-zipped CCS package file.

    \imageStyle{installer_file.png,width:20%}
    \image html installer_file.png "CCS Installer File"

- Follow the steps and at below screen, recommend to keep install directory as default.

    \imageStyle{install_directory.png,width:40%}
    \image html install_directory.png "CCS Install Path"

- Follow the steps and at below screen, recommend to keep setup type as "custom"

    \imageStyle{setup_type.png,width:50%}
    \image html setup_type.png "CCS Setup Type"
\cond SOC_AM243X || SOC_AM263X || SOC_AM263PX || SOC_AM273X || SOC_AM261X
- Follow the steps and at below screen, select the component as "Sitara AM2x MCUs" to install @VAR_SOC_NAME related emulation and GELs

    \imageStyle{select_components_sitara.png,width:50%}
    \image html select_components_sitara.png "CCS Select Components"
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

- Launch CCS and select the workspace.

    \imageStyle{first_launch.png,width:50%}
    \image html first_launch.png "CCS Select Workspace"

## Check Packages as seen by CCS {#CCS_PACKAGE_CHECK}

- Launch CCS

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

\cond SOC_AM263PX || SOC_AM261X
## Update CSP {#CSP_UPDATE}
\note AM263Px now supports one-click XIP .out application debugging with CSP 1.2.7. Refer to the guide below to update CSP and check for its availability.

- Goto "Help -> Check for updates"

    \imageStyle{ccs_update_check.png,width:30%}
    \image html ccs_update_check.png "Check for Updates Menu"

- The window will list the available updates. Select "Sitara device support" and click next.

    \imageStyle{ccs_update_avail.png,width:30%}
    \image html ccs_update_avail.png "List of available updates"

- Click on next.

    \imageStyle{ccs_update_details.png,width:30%}
    \image html ccs_update_details.png "Update details"

- Accept the licence agreement and click "Finish".

    \imageStyle{ccs_update_accept.png,width:30%}
    \image html ccs_update_accept.png "Review licenses"

- Restart CCS.

- Goto "Help -> About Code Composer Studio"

    \imageStyle{ccs_update_about.png,width:30%}
    \image html ccs_update_about.png "Help menu"

- Click on "Installation details".

    \imageStyle{ccs_update_inst_details.png,width:20%}
    \image html ccs_update_inst_details.png "About CCS"

- Scroll down to "Sitara device support". The CSP version should be 1.2.7 or above.

    \imageStyle{ccs_update_csp_version.png,width:40%}
    \image html ccs_update_csp_version.png "CSP version"

\note If the CSP changes are not taking effect, update the CSP timestamp.
\note Close CCS, go to "<ccs_insall_dir>\ccs\ccs_base\common\targetdb" and update the 'timestamp' file to current date and start the CCS
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
\cond SOC_AM263X
### AM263X-CC / AM263X-LP
\endcond
\cond SOC_AM263PX || SOC_AM261X
### AM263PX-CC
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

\cond SOC_AM263X
- In "Board or Device" type "@VAR_SOC_NAME" and select "AM263x"
\endcond
\cond SOC_AM263PX || SOC_AM261X
- In "Board or Device" type "@VAR_SOC_NAME" and select "AM263Px"
\endcond
    \imageStyle{ccs_target_config_00.png,width:50%}
    \image html ccs_target_config_00.png "Select @VAR_SOC_NAME EVM"

- For CCs using ISO7221ADR (U53) for JTAG, lower the TCLK to 1MHz. Under Advanced tab, in the Connection Properties
    - Select 'The JTAG TCLK Frequency (MHz)' as "Fixed with user specified value"
    - Select '--Enter a value from 100.0kHz to 5.5MHz' as "1MHz"

    \imageStyle{ccs_target_config_freq.PNG,width:50%}
    \image html ccs_target_config_freq.PNG "Configuring JTAG TCLK frequency"

- Under Advanced tab – Select Cortex_R5_0
  - Make sure the device csp gel file path is in the Initialization script field

    \imageStyle{ccs_target_config_gel.png,width:50%}
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