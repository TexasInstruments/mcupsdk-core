# Enabling Optishare In CCS {#OPTIFLASH_OPTISHARE_CCS}

[TOC]

# Introduction

This document goes over how to enable Optishare in System project in CCS.

# Enabling Optishare 

1. Import a system project. In this case, \ref EXAMPLES_DRIVERS_IPC_NOTIFY_ECHO is being imported. 

\imageStyle{optishare_ccs_project_load_0.png,width:40%}
\image html optishare_ccs_project_load_0.png "Importing of System Project"

2. Go to the properties of `ipc_notify_echo_am263px-cc_system_freertos_nortos` projects and open Optishare. The following is shown:

\imageStyle{optishare_ccs_properties_0.png,width:40%}
\image html optishare_ccs_properties_0.png "Optishare CCS Properties"

3. From here, Check the `Enable Optishare` checkbox.
4. After enabling, provide the Share Memory Specification File (\ref OPTIFLASH_OPTISHARE talks about this file).

\imageStyle{optishare_ccs_script_option_0.png,width:40%}
\image html optishare_ccs_script_option_0.png "Optishare Script option"

On this screen, there are other options as well.

**--nobss** will keep BSS section being shared and **--nodata** will keep data section from being shared. If Regional Address Translation (RAT) hardware is not present in the SOC for which Optishare is being applied, then enable both of these options for Optishare to work.

Keep Rest of the options same unless they are required to be changed.

# Building the Application

Once Optishare is enabled in the application, build it normally. This will generate all the necessary files. 

