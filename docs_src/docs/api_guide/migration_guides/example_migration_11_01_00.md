# Migrating examples to 11.01.00 from older versions {#EXAMPLE_MIGRATION_11_01_00}

[TOC]

\cond !SOC_AM64X
\note Images are shown for AM64x. It is application for @VAR_SOC_NAME as well.
\endcond

## Makefile Changes
### Library Name change on makefile and CCS projects
From 11.01.00 SDK all the libraries are built seperately for OS. There are seperate libraries available for NoRTOS and FreeROTS. 
So the makefiles needs to be updated accordingly. Please refer the sample changes on the makefile below. These changes are not applicbale for the 
librarries which were already built seperately for NoRTOS/FreeRTOS like kernel libraries. 

For NoRTOS/baremetal, 

\imageStyle{example_migration1.png,width:40%}
\image html example_migration1.png "Library name change for NoRTOS example"

For FreeRTOS, 

\imageStyle{example_migration2.png,width:40%}
\image html example_migration2.png "Library name change for FreeRTOS example"

similar change can be done on the CCS project as well

### OS define on makefile and CCS projects 
Addtional macro OS_NORTOS or OS_FREERTOS should be deined on the makefile or CC project based on the OS of the project. 

For NoRTOS/baremetal, 

\imageStyle{example_migration3.png,width:20%}
\image html example_migration3.png "OS Macro addition for NoRTOS example"

For FreeRTOS, 

\imageStyle{example_migration4.png,width:20%}
\image html example_migration4.png "OS Macro addition for FreeRTOS example"