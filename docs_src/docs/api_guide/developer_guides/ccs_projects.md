# Using SDK with CCS Projects {#CCS_PROJECTS_PAGE}

[TOC]

\note The steps on this page show screen shots and description based on Windows.
      However the steps in Linux would remain the same, unless mentioned otherwise.

\note By default, CCS import of SDK example projects performs a copy of example files to CCS workspace.
User are free to modify the example files without affecting the files in the SDK installed location.

\note Deleting the CCS project, deletes the project files from CCS workspace.
User should be careful especially when the copied example files are modified by user.

\note The screen shots shown on this page are for AM64x MCU+ SDK v7.3.0. You would see
      your specific SOC and SDK version that is installed.
      However the instructions and steps mentioned remain the same, unless mentioned otherwise.

## Introduction

All SDK examples can be built using CCS projects. Using CCS projects one can also
open SysConfig from within CCS itself to configure the example. This
section provides basic instructions and tips on using CCS projects.

\note To re-build libraries you need to use makefiles, see \ref MAKEFILE_BUILD_PAGE

## CCS Projectspec and CCS Projects

- When a example project is imported into CCS, what is really getting imported is a `projectspec` file.
- This file is a definition or blueprint of a CCS project, like compiler options, files to compile, libraries to link.
- CCS takes the projectspec as input and then generates CCS project files in the user workspace.
- Default workspace is,
  \code
  C:/Users/{user name}/workspace_v{CCS major version}/{project name}/
  \endcode
- The `projectspec` is also human readable, i.e you can open it in a text editor to see all the different
  options for that project.

##  Import a Project in CCS

- First open the "project explorer" as shown below

    \imageStyle{ccs_projects_00.png,width:20%}
    \image html ccs_projects_00.png "Project Explorer"

- "Right-click" in the project explorer window to import a project

    \imageStyle{ccs_projects_01.png,width:25%}
    \image html ccs_projects_01.png "Project Import"

\cond SOC_AWR294X || SOC_AM243X || SOC_AM64X || SOC_AM62X
- Click "Browse" and go to ${SDK_INSTALL_PATH}/examples/ folder

    \imageStyle{ccs_projects_02.png,width:40%}
    \image html ccs_projects_02.png "Select Examples Folder"

\endcond
\cond SOC_AM273X

- Click "Browse" and go to ${SDK_INSTALL_PATH}/examples/ folder
    \imageStyle{ccs_projects_02_am273x.png,width:40%}
    \image html ccs_projects_02_am273x.png "Select Examples Folder"

\endcond
\cond SOC_AM263X

- Click "Browse" and go to ${SDK_INSTALL_PATH}/examples/ folder
    \imageStyle{ccs_projects_02_am263x.png,width:40%}
    \image html ccs_projects_02_am263x.png "Select Examples Folder"

\endcond

\cond SOC_AWR294X || SOC_AM243X || SOC_AM64X || SOC_AM62X
- This will list all example projects, now you can select one or more or all projects to import
  into CCS.

    \imageStyle{ccs_projects_03.png,width:30%}
    \image html ccs_projects_03.png "Select Project"
\endcond
\cond SOC_AM273X
- This will list all example projects, now you can select one or more or all projects to import
  into CCS.

    \imageStyle{ccs_projects_03_am273x.png,width:30%}
    \image html ccs_projects_03_am273x.png "Select Project"
\endcond
\cond SOC_AM263X
- This will list all example projects, now you can select one or more or all projects to import
  into CCS.

    \imageStyle{ccs_projects_03_am263x.png,width:30%}
    \image html ccs_projects_03_am263x.png "Select Project"
\endcond

- All examples follow below naming convention to help you easily pick the example you need
    \code
    {example name}_{soc board}_{cpu}_{os}_{compiler toolchain}
    \endcode

- You can also navigate to the example folder to pick a specific example. All examples are organized as below in the examples folder.
    \code
    examples/{component or module}/{optional sub-module or sub-component}/
                   |
                   + -- {example name}/{board on which this example can run}/
                                         |
                                         + -- {cpu}_{os}/{compiler toolchain}
                                                                |
                                                                + -- example.projectspec  --> This is the file that CCS imports.
    \endcode

##  Browse a Project in CCS

\cond SOC_AWR294X || SOC_AM243X || SOC_AM64X || SOC_AM62X
- Once a example project is imported, you can learn more about the example, by clicking the "README.html" file

    \imageStyle{ccs_projects_04.png,width:50%}
    \image html ccs_projects_04.png "Example Readme"

\endcond
\cond SOC_AM273X
- Once a example project is imported, you can learn more about the example, by clicking the "README.html" file

    \imageStyle{ccs_projects_04_am273x.png,width:50%}
    \image html ccs_projects_04_am273x.png "Example Readme"

\endcond
\cond SOC_AM263X
- Once a example project is imported, you can learn more about the example, by clicking the "README.html" file

    \imageStyle{ccs_projects_04_am263x.png,width:50%}
    \image html ccs_projects_04_am263x.png "Example Readme"

\endcond

\cond SOC_AWR294X || SOC_AM243X || SOC_AM64X || SOC_AM62X
- You can also open the SysConfig GUI by double-clicking the "example.syscfg" file, (see also \ref SYSCONFIG_INTRO_PAGE)

    \imageStyle{ccs_projects_06.png,width:50%}
    \image html ccs_projects_06.png "Example SysConfig"
\endcond
\cond SOC_AM273X
- You can also open the SysConfig GUI by double-clicking the "example.syscfg" file, (see also \ref SYSCONFIG_INTRO_PAGE)

    \imageStyle{ccs_projects_06_am273x.png,width:50%}
    \image html ccs_projects_06_am273x.png "Example SysConfig"

\endcond
\cond SOC_AM263X
- You can also open the SysConfig GUI by double-clicking the "example.syscfg" file, (see also \ref SYSCONFIG_INTRO_PAGE)

    \imageStyle{ccs_projects_06_am263x.png,width:50%}
    \image html ccs_projects_06_am263x.png "Example SysConfig"

\endcond

- You can explore other project files by clicking on them.

##  Build a Project in CCS

- To build a project "right-click" on the project name and select "Build Project" to build it

\cond SOC_AWR294X || SOC_AM243X || SOC_AM64X || SOC_AM62X
- By default it builds in "debug" profile, i.e without compiler optimizations. To build with compiler
  optimizations, select "release" profile by clicking, "Build Configurations > Set Active > Release" as shown
  below.

    \imageStyle{ccs_projects_05.png,width:40%}
    \image html ccs_projects_05.png "Build a Project"
\endcond
\cond SOC_AM273X
- By default it builds in "debug" profile, i.e without compiler optimizations. To build with compiler
  optimizations, select "release" profile by clicking, "Build Configurations > Set Active > Release" as shown
  below.

    \imageStyle{ccs_projects_05_am273x.png,width:40%}
    \image html ccs_projects_05_am273x.png "Build a Project"

\endcond

\cond SOC_AM263X
- By default it builds in "debug" profile, i.e without compiler optimizations. To build with compiler
  optimizations, select "release" profile by clicking, "Build Configurations > Set Active > Release" as shown
  below.

    \imageStyle{ccs_projects_05_am263x.png,width:40%}
    \image html ccs_projects_05_am263x.png "Build a Project"

\endcond


- You can explore additional project options in the same menu.

## Build System Projects in CCS

- Some examples, to run correctly, need multiple binaries on different CPUs to be built, loaded and run.
  Such examples have "system projects" associated with them. One such example is inter-processor communication example,
  which shows multiple CPUs communicating among each other.

\cond SOC_AWR294X || SOC_AM243X || SOC_AM64X || SOC_AM273X

- System projects make it convenient to import one system project which in turn imports the grouped
  CPU specific projects.

    \imageStyle{ccs_projects_08.png,width:50%}
    \image html ccs_projects_08.png "System Project Import"
\endcond

\cond SOC_AM263X

- System projects make it convenient to import one system project which in turn imports the grouped
  CPU specific projects.

    \imageStyle{ccs_projects_08_am263x.png,width:50%}
    \image html ccs_projects_08_am263x.png "System Project Import"
\endcond

- System project have a naming convention as below
    \code
    {example name}_{soc board}_system_{os or tag}_{compiler toolchain}
    \endcode

- And the project file is located at
    \code
    examples/{component or module}/{optional sub-module or sub-component}/
                   |
                   + -- {example name}/{board on which this example can run}/
                                         |
                                         + -- system_{os or tag}/
                                                       |
                                                       + -- system.projectspec  --> This is the file that CCS imports.
    \endcode

\cond SOC_AWR294X || SOC_AM243X || SOC_AM64X || SOC_AM273X

- After import, one can browse and build the individual CPU specific projects as before
- You can also build the system project which will in turn build the grouped CPU projects.

    \imageStyle{ccs_projects_10.png,width:50%}
    \image html ccs_projects_10.png "System Project View"
\endcond

\cond SOC_AM263X

- After import, one can browse and build the individual CPU specific projects as before
- You can also build the system project which will in turn build the grouped CPU projects.

    \imageStyle{ccs_projects_10_am263x.png,width:50%}
    \image html ccs_projects_10_am263x.png "System Project View"
\endcond

\cond SOC_AWR294X || SOC_AM243X || SOC_AM64X || SOC_AM273X

- You can also launch SysConfig GUI using the individual projects that were imported as part of the "system" project. However now, SysConfig will show the configuration of all CPUs in a single SysConfig window. (see also \ref SYSCONFIG_INTRO_PAGE)

    \imageStyle{ccs_projects_11.png,width:80%}
    \image html ccs_projects_11.png "SysConfig view with System Projects"

\endcond

\cond SOC_AM263X

- You can also launch SysConfig GUI using the individual projects that were imported as part of the "system" project. However now, SysConfig will show the configuration of all CPUs in a single SysConfig window. (see also \ref SYSCONFIG_INTRO_PAGE)

    \imageStyle{ccs_projects_11_am263x.png,width:80%}
    \image html ccs_projects_11_am263x.png "SysConfig view with System Projects"

\endcond

\cond SOC_AM64X
## 1 Click Debug with CCS Projects {#CCS_PROJECTS_1CLICK_DEBUG}
 - Projects imported in CCS can also be loaded and run on the EVM with single click. Please note this uses SOC initialization method \ref EVM_SOC_INIT
 - Please make sure you have completed the steps required mentioned in \ref PREREQUISITES

 - Import the project as shown below
    \imageStyle{1click_debug_01.PNG,width:40%}
    \image html 1click_debug_01.PNG "Import Project"

    \imageStyle{1click_debug_02.PNG,width:50%}
    \image html 1click_debug_02.PNG "Browse And Select Project "

 - As 1 click debug works with SOC initialization method, please remove gel file paths from the default target configuration
   "..\..\emulation\gel\AM64x\CPU_reset.gel" and Save.
    \imageStyle{1click_debug_03.PNG,width:60%}
    \image html 1click_debug_03.PNG "Remove Gel Reference From Default Target Configuration"

 - Right click on the project and select the Code Composer debug session as shown below
    \imageStyle{1click_debug_04.PNG,width:60%}
    \image html 1click_debug_04.PNG "Code Composer Debug Session"

 - Select the Core to run
    \imageStyle{1click_debug_05.PNG,width:40%}
    \image html 1click_debug_05.PNG "Core to Run"

 - This will build the example internally as shown below
    \imageStyle{1click_debug_06.PNG,width:50%}
    \image html 1click_debug_06.PNG "Console Output Build Example"

 - The target configuration is launched and the example is loaded on the core and halts in Main
    \imageStyle{1click_debug_07.PNG,width:50%}
    \image html 1click_debug_07.PNG "Example load"

 - Click on the resume to run the example
    \imageStyle{1click_debug_08.PNG,width:50%}
    \image html 1click_debug_08.PNG "Example run"

 - The program output will be seen on CCS console, and/or UART terminal, if enabled.
   Below shows a sample output on both CCS console and UART console, after running the "hello world" program.

    \imageStyle{1click_debug_09.PNG,width:50%}
    \image html 1click_debug_09.PNG "Run the Program"

\endcond

\cond SOC_AM243X
## 1 Click Debug with CCS Projects {#CCS_PROJECTS_1CLICK_DEBUG}
 - Projects imported in CCS can also be loaded and run on the EVM with single click. Please note this uses SOC initialization method \ref EVM_SOC_INIT
 - Please make sure you have completed the steps required mentioned in \ref PREREQUISITES

 - Import the project as shown below
    \imageStyle{1click_debug_01.PNG,width:40%}
    \image html 1click_debug_01.PNG "Import Project"

    \imageStyle{1click_debug_02.PNG,width:50%}
    \image html 1click_debug_02.PNG "Browse And Select Project "

 - As 1 click debug works with SOC initialization method, please remove gel file paths from the default target configuration
   "..\..\emulation\gel\AM24x\CPU_reset.gel" and Save.
    \imageStyle{1click_debug_03.PNG,width:60%}
    \image html 1click_debug_03.PNG "Remove Gel Reference From Default Target Configuration"

 - Right click on the project and select the Code Composer debug session as shown below
    \imageStyle{1click_debug_04.PNG,width:60%}
    \image html 1click_debug_04.PNG "Code Composer Debug Session"

 - Select the Core to run
    \imageStyle{1click_debug_05.PNG,width:40%}
    \image html 1click_debug_05.PNG "Core to Run"

 - This will build the example internally as shown below
    \imageStyle{1click_debug_06.PNG,width:50%}
    \image html 1click_debug_06.PNG "Console Output Build Example"

 - The target configuration is launched and the example is loaded on the core and halts in Main
    \imageStyle{1click_debug_07.PNG,width:50%}
    \image html 1click_debug_07.PNG "Example load"

 - Click on the resume to run the example
    \imageStyle{1click_debug_08.PNG,width:50%}
    \image html 1click_debug_08.PNG "Example run"

 - The program output will be seen on CCS console, and/or UART terminal, if enabled.
   Below shows a sample output on both CCS console and UART console, after running the "hello world" program.

    \imageStyle{1click_debug_09.PNG,width:50%}
    \image html 1click_debug_09.PNG "Run the Program"

\endcond

\cond SOC_AM273X
## 1 Click Debug with CCS Projects {#CCS_PROJECTS_1CLICK_DEBUG}

 - Projects imported in CCS can be loaded and run on the EVM with single click.

 - Right click on the project and select the Code Composer debug session as shown below
    \imageStyle{1click_debug_01.png,width:40%}
    \image html 1click_debug_01.png "Code Composer Debug Session"

 - Select the Core to run
    \imageStyle{1click_debug_02.png,width:40%}
    \image html 1click_debug_02.png "Core to Run"

 - This will build the example internally as shown below
    \imageStyle{1click_debug_03.png,width:40%}
    \image html 1click_debug_03.png "Console Output Build Example"

 - The target configuration is launched and the example is loaded on the core and halts in Main
    \imageStyle{1click_debug_04.png,width:40%}
    \image html 1click_debug_04.png "Example load"

 - Click on the resume to run the example
    \imageStyle{1click_debug_05.png,width:40%}
    \image html 1click_debug_05.png "Example run"

 - The program output will be seen on CCS console, and/or UART terminal, if enabled.
   Below shows a sample output on both CCS console and UART console, after running the "hello world" program.

    \imageStyle{ccs_load_run_08.png,width:50%}
    \image html ccs_load_run_08.png "Run the Program"

\endcond

\cond SOC_AM263X
## 1 Click Debug with CCS Projects {#CCS_PROJECTS_1CLICK_DEBUG}

 - Projects imported in CCS can be loaded and run on the EVM with single click.

 - Right click on the project and select the Code Composer debug session as shown below
    \imageStyle{1click_debug_01.png,width:40%}
    \image html 1click_debug_01.png "Code Composer Debug Session"

 - Select the Core to run
    \imageStyle{1click_debug_02.png,width:40%}
    \image html 1click_debug_02.png "Core to Run"

 - This will build the example internally as shown below
    \imageStyle{1click_debug_03.png,width:40%}
    \image html 1click_debug_03.png "Console Output Build Example"

 - The target configuration is launched and the example is loaded on the core and halts in Main
    \imageStyle{1click_debug_04.png,width:40%}
    \image html 1click_debug_04.png "Example load"

 - Click on the resume to run the example
    \imageStyle{1click_debug_05.png,width:40%}
    \image html 1click_debug_05.png "Example run"

 - The program output will be seen on CCS console, and/or UART terminal, if enabled.
   Below shows a sample output on both CCS console and UART console, after running the "hello world" program.

    \imageStyle{1click_debug_06.png,width:50%}
    \image html 1click_debug_06.png "Run the Program"



\endcond


\cond SOC_AWR294X || SOC_AM263X || SOC_AM243X || SOC_AM64X || SOC_AM62X
## Load and Run Executables Built with CCS Projects

- Simply load and run binary .out on EVM using CCS (see \ref CCS_LAUNCH_PAGE)

- In case of "system" examples, you need to load the binaries for each CPU one by one in CCS and then run each CPU.
  Typically the order of load and run of different CPUs is not important.

\endcond

## Delete a Project in CCS

- To delete a imported project "right-click" on the project name and select "Delete" to delete it.

\cond SOC_AWR294X || SOC_AM243X || SOC_AM64X || SOC_AM273X

- It is recommended to check the box as shown below, else you will have trouble re-importing the project

    \imageStyle{ccs_projects_07.png,width:50%}
    \image html ccs_projects_07.png "Delete a Project"
\endcond

\cond SOC_AM263X

- It is recommended to check the box as shown below, else you will have trouble re-importing the project

    \imageStyle{ccs_projects_07_am263x.png,width:50%}
    \image html ccs_projects_07_am263x.png "Delete a Project"
\endcond

- Note, that deleting a project, only deletes it from your CCS workspace, the .c, .h and other project files
  are located with the ${SDK_INSTALL_PATH}/examples and these are not deleted.

- You can reimport a project later as needed.
