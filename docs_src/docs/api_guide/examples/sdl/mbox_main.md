# SDL MBOX {#EXAMPLES_SDL_MBOX_MAIN}

[TOC]

# Introduction

This example demonstrates the usage of the SDL MBOX module. The example shows how to setup and use the ECC BUS Safety Diagnostic operation.
Shows the generation of SEC, DED and RED error on MSS MBOX, DSS MBOX and RSS MBOX

# Note
Note : SEC - Single Error Correction, DED - Double Error Correction, RED - Redundancy Error Correction

AM263x  : Supports MSS MBOX
AM273X  : Supports MSS MBOX and DSS MBOX
AWR294x : Supports MSS MBOX, DSS MBOX and RSS MBOX

Use Cases On R5F Core
---------

 Use Case | Description
 ---------|------------
 UC-1     | SEC Error insertion on RSS MBOX.
 UC-2     | DED Error insertion on RSS MBOX.
 UC-3     | RED Error insertion on RSS MBOX.

Use Cases On C66 Core
---------

 Use Case | Description
 ---------|------------
 UC-1     | SEC Error insertion on DSS MBOX.
 UC-2     | DED Error insertion on DSS MBOX.
 UC-3     | RED Error insertion on DSS MBOX.
 UC-4     | SEC Error insertion on RSS MBOX.
 UC-5     | DED Error insertion on RSS MBOX.
 UC-6     | RED Error insertion on RSS MBOX.

# Supported Combinations {#EXAMPLES_SDL_MBOX_MAIN_COMBOS}

\cond SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | R5F, c66  nortos
 Toolchain      | ti-arm-clang,ti-c6000
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/mbox/mbox_main/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref SDL_MBOX_PAGE

# Sample Output

Shown below is a sample output when the application is run on  On R5F Core,

\code
 MBOX Application

MBOX TEST START : starting

MSS_MBOX_SEC_Test_init: Init ESM complete

ESM Call back function called and action taken

MSS_MBOX_DED_Test_init: Init ESM complete

ESM Call back function called and action taken

MSS_MBOX_RED_Test_init: Init ESM complete

ESM Call back function called and action taken

Applications Name: MSS_MBOX_SecTest  PASSED

Applications Name: MSS_MBOX_DedTest  PASSED

Applications Name: MSS_MBOX_RedTest  PASSED


 All tests have passed
\endcode

Shown below is a sample output when the application is run on  On C66 Core,

\code
 MBOX Application

 MBOX TEST START : starting

 Applications Name: DSS_MBOX_SecTest  PASSED

 Applications Name: DSS_MBOX_DedTest  PASSED

 Applications Name: DSS_MBOX_RedTest  PASSED

 Applications Name: RSS_MBOX_SecTest  PASSED

 Applications Name: RSS_MBOX_DedTest  PASSED

 Applications Name: RSS_MBOX_RedTest  PASSED

 All tests have passed
\endcode


