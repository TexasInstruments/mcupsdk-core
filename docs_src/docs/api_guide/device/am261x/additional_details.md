#  Additional Details {#ADDITIONAL_DETAILS_PAGE}

[TOC]


### Alternate Method to Build and Run the examples

\note These steps are optional.

Below method cab be used for running examples built with makefiles

 - **Step 1:** Build a "hello world" example for the EVM, see \subpage GETTING_STARTED_BUILD

 - **Step 2:** Load and run the "hello world" example on the EVM, see \subpage CCS_LAUNCH_PAGE

 - **Step 3:** Flash the "hello world" example on the EVM and boot without CCS, see \subpage GETTING_STARTED_FLASH

#### SOC Initialization {#EVM_SOC_INIT}

Before any program can be loaded and run on the EVM, the SOC needs to be initialized.
Below sections describes the various options available for SOC initialization.

#### SOC Initialization Using CCS Scripting {#EVM_SOC_INIT_NOBOOT_MODE}

##### Set Environment Variable

\note This step needs to be done once and is needed for the
  SOC initialization script `load_sbl.js` to find certain initialization files within the SDK folder.
  This variable is not used otherwise in the build process. If you dont like adding variables in the environment, then
  you need to edit the file `${SDK_INSTALL_PATH}/tools/ccs_load/am261x/load_sbl.js` and specify the SDK path in the file itself.

- Add path to the SDK folder as a environment variable in the host machine.

- In windows, goto "Windows Task Bar Search" and search for "environment variables for your account"
        \imageStyle{ccs_setup_03.png,width:15%}
        \image html ccs_setup_03.png "Environment Variables For Your Account"

- Add a new variable named `MCU_PLUS_SDK_AM261X_PATH` and point it to the path where the SDK is installed

\imageStyle{ccs_setup_04.png,width:50%}
\image html ccs_setup_04.png "Add New Environment Variable For Your Account"

- In Linux, usually you need to add a line as below in the ${HOME}/.bashrc,

        export MCU_PLUS_SDK_AM243X_PATH=${HOME}/ti/mcu_plus_sdk_am261x_{sdk version}/

- If CCS is open, close and reopen CCS for the CCS to be able to see the updated environment variable

##### Run the SOC Initialization Script
###### AM261X-LP

###### AM261X-CC

#### SOC Initialization using the Binary Flashed in OSPI memory {#EVM_FLASH_SOC_INIT}
### AM261X-CC

#### Run the example


