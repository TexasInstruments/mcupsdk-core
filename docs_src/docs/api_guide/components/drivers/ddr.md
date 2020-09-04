# DDR {#DRIVERS_DDR_PAGE}

[TOC]

The DDR driver provides API to configure the DDR module.

## Features Supported

- Ability to enable/disable DDR support in your application
- Ability to specify the DDR config generated using the "DDR SUBSYSTEM REGISTER CONFIGURATION" tool
- Ability to enable/disable inline ECC (1b and 2b) in the application
- Add regions for enabling inline ECC

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

- Ability to specify the DDR config generated using the "DDR SUBSYSTEM REGISTER CONFIGURATION" tool
- Enables DDR, EMIF module clock when DDR module is selected
- Toggle inline ECC functionality
- Add regions for which inline ECC needs to be enabled

## Features not Supported

NA

## Creating your own DDR config file

### Step 1: Generate a DDR config file for your EVM

- Goto https://dev.ti.com/sysconfig
- Select "Software Product" as "DDR SUBSYSTEM REGISTER CONFIGURATION"
- Select "Device" of interest, @VAR_SOC_NAME in this case.

\imageStyle{ddr_config_tool.png,width:60%}
\image html ddr_config_tool.png "DDR SUBSYSTEM REGISTER CONFIGURATION Tool"

- Configure the DDR parameters using the tool
- Save the generated `board_ddrReginit.h` in your project workspace or work area

\imageStyle{ddr_config_tool_save.png,width:90%}
\image html ddr_config_tool_save.png "Configure and save the generated file"

### Step 2: Add the generated DDR config file to your project

- Open SysConfig (\ref SYSCONFIG_INTRO_PAGE) for your project
- Add "DDR" as shown below

\imageStyle{ddr_sysconfig.png,width:60%}
\image html ddr_sysconfig.png "Add DDR via SysConfig"

- Specify the path to the generated `board_ddrReginit.h` file including the filename in the sysconfig text box
  as shown above

  - Make sure to use forward slash "/" in the file path so that this will work with linux as well as windows build
  - Make sure that path to this is file set in your application include path, as needed.

- Save the sysconfig project and build your application

## Creating your own LPDDR config file

### Step 1: Generate a LPDDR config file for your EVM

- Goto https://dev.ti.com/sysconfig
- Select "Software Product" as "DDR SUBSYSTEM REGISTER CONFIGURATION"
- Select "Device" of interest, @VAR_SOC_NAME in this case.

\imageStyle{ddr_config_tool.png,width:60%}
\image html ddr_config_tool.png "DDR SUBSYSTEM REGISTER CONFIGURATION Tool"

- Configure the LPDDR parameters using the tool
- Save the generated `board_ddrReginit.h` in your project workspace or work area. Rename the file as "board_lpddrReginit.h".

\imageStyle{lpddr_config_tool_save.png,width:90%}
\image html lpddr_config_tool_save.png "Configure and save the generated file"

### Step 2: Add the generated LPDDR config file to your project

- Open SysConfig (\ref SYSCONFIG_INTRO_PAGE) for your project
- Add "DDR" as shown below

\imageStyle{lpddr_sysconfig.png,width:60%}
\image html lpddr_sysconfig.png "Add DDR via SysConfig"

- Specify the path to the generated `board_lpddrReginit.h` file including the filename in the sysconfig text box
  as shown above

  - Make sure to use forward slash "/" in the file path so that this will work with linux as well as windows build
  - Make sure that path to this is file set in your application include path, as needed.

- Save the sysconfig project and build your application

## Important Usage Guidelines

\cond SOC_AM64X || SOC_AM243X
- Please read the application note https://www.ti.com/lit/pdf/spracu1
  for projects having own board design with DDR.
\endcond
- Ideally DDR should be enabled in the bootloader application and the SDK bootloader by default
  enables DDR by default.

## Usage Overview

Include the below file to access the APIs
\snippet Ddr_sample.c include

Initialize DDR
\snippet Ddr_sample.c init_ddr

## Example Usage

NA

## API

\ref DRV_DDR_MODULE