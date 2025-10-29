#  Download, Install and Setup IAR Embedded Workbench {#IAR_SETUP_PAGE}

[TOC]

\note The steps on this page need to be done once on a given host machine
\cond SOC_AM263X || SOC_AM261X 
\endcond
## Download IAR Embedded Workbench

- Download IAR @VAR_IAR_ARM_VERSION, https://www.iar.com/embedded-development-tools/iar-embedded-workbench

## Install IAR Embedded Workbench

- Install IAR Embedded Workbench @VAR_IAR_ARM_VERSION by double clicking the downloaded installer file **ewarm-@VAR_IAR_ARM_VERSION.exe**.

- Follow the steps and at below screen, recommend to keep install directory as default.
    \imageStyle{iar_install_directory.png,width:40%}
    \image html iar_install_directory.png "IAR Embedded Workbench Install Path"


- Follow the installer instructions until the installation is complete.
- Once installation is completed, Launch IAR Embedded Workbench IDE.
- To configure your IAR License, navigate to the Toolbar, then select `Help` > `License Manager`

## Configure Sysconfig to IAR Embedded Workbench IDE {#IAR_SYSCFG_SETUP}
- Navigate to `Tools` > `Configure Viewers`
- Click the **Import** button
- Navigate to SDK directory
- Select the configuration file located at `{SDK_INSTALL_PATH}/tools/iar/sysconfig_iar_setup.xml`
- Click OK to save changes
    \imageStyle{iar_config_viewer_menu.png,width:30%}
    \image html iar_config_viewer_menu.png "IAR Configure Viewer"

## Setup IAR EW
- Refer for further steps to setup IAR EW \ref IAR_PROJECTS_PAGE

## Known Issues

### IAR Embedded Workbench Does Not Support GEL Scripts {#IAR_SBLL_NULL_LOAD}
- IAR Embedded Workbench does not natively support the execution of GEL scripts. These scripts are used by Code Composer Studio (CCS) to initializing SOC, clocks and other registers during the startup of debugging session.

**Workaround** - Use SBL NULL for device Initialization

- Flash the SBL NULL binary to the target board before loading your application program or starting the debug session.
- The SBL NULL bootloader performs essential device initialization steps similar to what a GEL scripts do.
- Refer \ref EXAMPLES_DRIVERS_SBL_NULL

### Debugging Multi-core Projects Using the XDS110 in IAR {#IAR_MULTI_CORE_LOAD}
- IAR Embedded Workbench lacks built-in support for loading and simultaneously debugging multi-core processors when using TI's XDS110 debug probe.

**Workaround** - Use Code Composer Studio (CCS) for Loading and Debugging

- The most recommended solution is to use  Code Composer Studio (CCS) for the loading and debugging, even if you prefer to build your multi-core projects in IAR.
- Build your multi-core project within IAR Embedded Workbench to generate the executable output file (`.out`).
- Switch to Code Composer Studio to load `.out` file onto the target device.
- Refer \ref CCS_LOAD_RUN


