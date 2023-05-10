# k3-respart-tool

Resource Partitioning tool for K3 devices. This tool is based on Texas
Instrument's SysConfig tool. It allows you to configure various system level
parameters and generate the data which can be fed into many software components.

Typical usage for this tool is for System integrators, where one would be  able
to partition **various resources** across different software components. These
resources includes DMA channels, rings, proxies, interrupts, etc. Apart from
this, the tool supports configuration of QoS (Quality of Service) and Firewall
parameters which helps in ensuring partitioning of **peripheral devices** across
different CPUs or virtual machines.

## Getting started

To start using this tool, follow these instructions:
* Download the SysConfig tool from [SysConfig release download link]
(https://www.ti.com/tool/download/SYSCONFIG)
* Download the file setup.exe or setup.run for Windows or Linux machine
respectively.
* If you have the **k3-resource-partitioning** packaged in the SDK, use this or
clone from TI internal [bitbucket project]
(https://bitbucket.itg.ti.com/projects/PSDKLA/repos/k3-resource-partitioning/browse)
* Run the **setup script** `./scripts/setup.py -s /path/to/sysconfig/installation`
to patch the SysConfig tool.
* Open the SysConfig tool GUI from the desktop shortcut and select the
**software product** by navigating to the path where the
**k3-resource-partitioning** is available.
* Lastly, click on **Browse** button to open existing design for the platform
that you are interested in. Navigate to the out/ directory and you will find
baseline files for your platform. Use this as the starting point for any
customization.
* We do not recommend you to start from scratch, always load the baseline file
out/XYZ-platform-name.syscfg

For devices that are not yet officially supported by the latest SysConfig tool,
Resource Partitioning tool may provide patch files to support the device in 
SysConfig. Run the **sysconfig-deviceData-update** script 
`./scripts/sysconfig-deviceData-update.sh <soc> 
--sysconfig_dir=/path/to/sysconfig/installation` to patch the SysConfig tool.

## Troublshooting

* If you get an error like **No product with name "K3-Respart-Tool" and version
0.5 found**, Most likely you have missed the step to select the
**software product**. * If you are not able to see the HTML table and getting
error **ommitted HTML**, Most likely you may have missed / failed to run the
**setup script**. * If you see error **XYZ is not generated when configuration
errors exist**, It means your resource allocation has few errors. Please adjust
the allocation to fix the errors and then you can generate the output files.

## Usage

Once you have loaded the product in the SysConfig, and opened the baseline file,
you can accomplish following things.

### NAVSS Resource partitioning

This module allows to partition or allocate Navigator Subsystem (NAVSS)
resources across different hosts in the System. A **host** is any software
component which has a dedicated context for communication with
**System firmwarem** (SYSFW). A **Resource Management board config** file
describes how these resources are partitioned. This board config is passed by
the bootloader to System firmware as part of the boot up sequence. This module
allows you to automatically generate this file in an attempt to ease the NAVSS
resource partitioning.

In the left pane of GUI, you will see different hosts available and each host
describes the resources allocated to it. All the NAVSS resources are organized
in different **groups**, where user can specify the required **count** for each
resource. The tool automatically allocates the resources taaking into
consideration all the counts for all the hosts.

Apart from the resource allocation, the tool also has an option to configure
different **host capabilities**. Click on the **?** sign next to the host name
to read the detailed documentation for that host module.

#### Review resource allocation

At any point, user can review the current resource allocation done by the tool.
Select the pane **Resource Allocation Table** from the three dots shown at the
top right corner of the tool. Here, it presents an HTML table which shows the
exact range of resources allocated for each host. Each column describes the
resource ranges assigned for a certain host. Each row describes how different
host consume the give resource.

### QoS configuration

This module allows to generate the data required for configuring CBASS QoS
(Quality of Service) endpoints. QoS configuration includes two types of
parameters. Few parameters are used for tuning the performance of the DMA
transactions in the interconnect, and others are used for setting up the
IOMMU paths for the masters. For each device, there are multiple master ports
using which, DMA requests are made. Each device is capable of driving different
values of channel_id along with the DMA request. For every channel, a unique QoS
configuration can be programmed.

In the GUI, User should select a device, choose the endpoints, and select a list
of channels for which QoS should be programmed. Note that it is possible to add
multiple instances of QoS module with same device as long as the endpoints and
channels do not overlap. The tool auto generates a simple address-value pair
data structure in the **qos-config.c** output file. This can be used by any
software (typically bootloader) to program all the QoS settings.

### Firewall configuration

This module allows to generate the data required for Firewall configuration.
This is very much useful to ensure peripheral partitioning across CPU cores.
User can describe if a certain peripheral needs to be accessible ONLY from a
list of certain hosts. Each peripheral is protected by a set of firewalls. User
can select what all firewalls need to be programmed. Each firewall has different
numbers of regions availalbe. The region describes the address range where the
filter rules are applicable. Each region can be programmed to define access
rules for upto 3 permission slots. Multiple CPUs can have the same priv_id and
this would means one slot should be sufficient. The permission slot describes
if a transaction from a CPU cores with slected priv_id is allowed or not. You
can define this for different values of security, privilege level and
transaction types. e.g. Allow Secure writes, Secure reads, but do not allow Non
secure writes from A72 to MMC.

By default, the tool will automatically set the required start/end addresses to
be configured in the region, optionally allowing the user to define custom
region if required. Also, the tool allows to select a host_id and populates the
priv_id automatically. Using this data, it generates an array of data TISCI
message data structure that can be directly passed to SYSFW for firewall
configuration.

#### Split Resource Management Board Configuration

In order to reduce RM boardconfig size on TIFS data, user can filter some resources
those are not applicable for TIFS. The resource types those can be removed are 
present in **tifsResRem.json**. While generating **tifs-rm-cfg.c** tool will
skip all the resource that are mentioned in the **tifsResRem.json.

## Generate output files

This tool generates different files which has the RM board config data, QoS
configuration and Firewall configuration. Following table describes how to use
these files:

| Filename                       | Used by       | Output destination                   | Comments                                                                    |
|--------------------------------|---------------|--------------------------------------|-----------------------------------------------------------------------------|
| rm-cfg.c                       | k3-image-gen  | soc/<soc>/<profile>/                 | e.g. - k3-image-gen/soc/j721e/evm/rm-cfg.c                                  |
| tifs-rm-cfg.c                  | k3-image-gen  | soc/<soc>/<profile>/                 | e.g. - k3-image-gen/soc/j721e/evm/tifs-rm-cfg.c                             |
| sysfw_img_cfg.h                | k3-image-gen  | soc/<soc>/<profile>                  | e.g. - k3-image-gen/soc/j721e/evm/sysfw_img_cfg.h                           |
|                                |               |                                      |                                                                             |
| <soc>-qos-config.c             | U-boot        |                                      |                                                                             |
| <soc>-qos-config.c             | SBL           |                                      |                                                                             |
|                                |               |                                      |                                                                             |
| <soc>-firewall-config.c        | U-boot        |                                      |                                                                             |
| <soc>-firewall-config.c        | SBL           |                                      |                                                                             |
|                                |               |                                      |                                                                             |
| sciclient_defaultBoardcfg.c    | PDK sciclient | packages/ti/drv/sciclient/soc/V<X>/  | e.g. - pdk/packages/ti/drv/sciclient/soc/V1/sciclient_defaultBoardcfg.c     |
| sciclient_defaultBoardcfg_rm.c | PDK sciclient | packages/ti/drv/sciclient/soc/V<X>/  | e.g. - pdk/packages/ti/drv/sciclient/soc/V1/sciclient_defaultBoardcfg_rm.c  |
| sciclient_tifsBoardcfg_rm.c    | PDK sciclient |                                      |                                                                             |
   
## Developer notes

### directory structure
* **.metadata/product.json** - This file describes all the components that needs
to be loaded by the SysConfig tool. It also describes the supported platforms.
`product.json` describes the components and the components describe the list of
modules and templates that are applicable for the selected device.
* **modules** - This contains the UI module definition for different
configurables, their grouping, organization, and the Javascripts for handling
onChange events and validations.
* **templates** - This contains the xdt files which describe the output file
formats and small code snippets to generate the data using templates. It also
has few views to  describe the data in a more visual format like a Markdown
table or HTML table.
* **scripts** - These are the Javascripts for parsing different input data files
which generate the SoC specific JSON objects for the usage in the modules.
It also implements few utility functions which are frequently called by modules
and templates.
* **data** - This contains the JSON objects that the tool uses to populate the
UI items with SoC specific data. Many of these are auto generated using the
parsing scripts.
* **deviceData** - This contains Sysconfig patch files for devices that are
not yet officially supported in the Sysconfig Tool.

### Updating the device data

This tool needs lot of hardware and software data to present the GUI and to
allocate resources. Majority of this data is auto generated using SoC JSON files,
System firmware public documentation, etc. Most users do not need to update this
data. This is required only when adding support for a new SoC, migrating to new
SYSFW version or migrating to new SoC JSON, etc.

Make sure to add symbolic links to the appropriate repositories as shown below:
```
#  Do this ONLY if you want to update the data for SysConfig tool
dir=`git rev-parse --show-toplevel`
ln -s <path/to/system firmware repo> $dir/../system-firmware
ln -s <path/to/system firmware autogen repo> $dir/../system-firmware

```

There is a utility script `scripts/generate-data.sh [SOCNAME | all]` to generate
all the required JSON objects for selected SoC or for all of them.


### Updating ResDependencies.json  

ResDependencies.json file contains extra restrictions on the resource allocation.
For example all the interrupts may not be available for all the cores. this
information is added in this file. Basic structure of the file can be created
using the autogen script itself(by invoking the gen_initial_user_json function
in generate-data.sh)

To add interrupt related updates, look for interrupt_cfg.rst file in the folder
docs/public/5_soc_doc/<soc> in system-firmware repo. In that
file, search for tables having its name ending with "Output Destinations".
Each table represents one type of resource and its interrupt routing information.
These resources include CMP_EVENT_INTROUTER0, MAIN_GPIOMUX_INTROUTER0,
MCU_MCU_GPIOMUX_INTROUTER0, TIMESYNC_EVENT_INTROUTER0 etc. Ignore the 
TIMESYNC_EVENT_INTROUTER0 and CMP_EVENT_INTROUTER0 resource's table and look for
all other resource's table. Take one resource's table and look for the columns
"Destination Name" and "IR Output Index". One entry in the table represents that
the IR Output Index can be routed to the corresponding host. Select one host
from the "Destination Name" column and see which all "IR Output Index" can be
routed to the host. The IR Output Index might not be contiguous, but to avoid
further complexity, take the largest continuous range of output index that can
be assigned to the host. Similarly check for other hosts and find what ranges of
resource can be assigned to the host. Add this information as a "resRange" entry
with the following fields resStart: the starting index of resource, resCount:
number of resources that can be assigned,restrictHosts: the hosts that can be
assigned the particular range of resource for the corresponding resource, in
ResDependebcies.json file.
For example:
```
    "deviceName": "AM62X_DEV_WKUP_MCU_GPIOMUX_INTROUTER0",
    "subtypeName": "RESASG_SUBTYPE_IR_OUTPUT",
    "utype": "WKUP GPIO Interrupt Router",
    "resRange": [
        {
            "resStart": 6,
            "resCount": 4,
            "restrictHosts": [
                "A53_0",
                "A53_1",
                "A53_2",
                "A53_3"
            ]
        }
    ]
```
when assigning RESASG_SUBTYPE_IR_OUTPUT for A53, resource range from 6 to 4 nos
is a valid range. Adding this JSON entry will ensure that the tool can take care
of the custom range allocation

Similarly search for table's name ending with "Interrupt Destinations" in
interrupt_cfg.rst file mentioned in the above. These includes
DMASS0_INTAGGR_0 Interrupt Aggregator Virtual Interrupt Destinations,
SA3_SS0_INTAGGR_0 Interrupt Aggregator Virtual Interrupt Destinations etc.
These table represent resources related to  Virtual Interrupt. Do the same
procedure for these resources also as mentioned in the above paragraph.
