# Modifying resource allocation {#RESOURCE_ALLOCATION_GUIDE}

[TOC]

## Introduction

Various resources of the SOC like the number of DMA channels, number of interrupt router outputs, number of interrupt aggregator virtual interrupt numbers etc. are usually managed by a resource management system or a resource manager.
In the case of AM64x and AM243x devices, this is managed by the System Controller Firmware (SYSFW) running on the M3 core. Once the SYSFW is loaded on M3 and is initialized, we need to send a certain configuration data to the SYSFW regarding the resources we would be using. This is largely an array of resource assignment entries, with each entry specifying the start number of the resource, count or number of resource needed, type of resource, host id of the core which will request for this resource, etc. Later when the request for a specific resource is made, the SYSFW will cross check the request parameters with this already sent configuration data, and the requested resources will only be allocated if that falls within the range in this configuration data. We call this the Resource Management Board Configuration or __RM boardcfg__.

## Changing a particular resource
\note At the moment the changes need to made manually by editing C files, but soon a Sysconfig GUI based tool will be available to generate this C file.

- To change the assigned number of a particular resource, the `sciclient_defaultBoardcfg_rm.c` file at path `{SDK_ROOT_DIRECTORY}\source\drivers\sciclient\sciclient_default_boardcfg\am64x_am243x\` location.

- Around line 135 of the file, you should find a line which says `.resasg_entries = {`. This is an array of resource assignment entries. Resource assignment entries are structs with members
 - num_resource (Number or Count of resources)
 - type (Type of resource, can be DMA channel, or Interrupt Router Output, etc.)
 - start_resource (Starting number of the resource)
 - host_id (Host ID of the core which can request for this resource)

\imageStyle{resasg_1.png,width:50%}
\image html resasg_1.png "Resource Assignment Entries"

- Once you have identified the resource type which you want to modify, you can see if an entry with that type is already part of the array. If yes, you can go ahead and make the changes directly depending on what you want to do. Change the start number, decrease/increase the number of resources etc. Make sure that resources do not overlap when you change this. Another thing to take care is the host_id in the entry you make. If you add the entry with the current resource range but with a wrong host_id, your host won't be able to request for that particular resource. For each core present in the SoC, there will be two host types - Secure and Non-Secure. These host ids are defined in `{SDK_ROOT_DIRECTORY}\source\drivers\sciclient\include\tisci\am64x_am243x\tisci_hosts.h` file.

- Here is the host id to core mapping:
<table>
<tr>
    <th>HOST ID
    <th>Core
</tr>
<tr>
    <td>TISCI_HOST_ID_DMSC (0U)
    <td>DMSC ARM Cortex M3
</tr>
<tr>
    <td>TISCI_HOST_ID_MAIN_0_R5_0 (35U)
    <td>Cortex R5FSS0_0 (Secure Context)
</tr>
<tr>
    <td>TISCI_HOST_ID_MAIN_0_R5_1 (36U)
    <td>Cortex R5FSS0_0 (Non-Secure Context)
</tr>
<tr>
    <td>TISCI_HOST_ID_MAIN_0_R5_2 (37U)
    <td>Cortex R5FSS0_1 (Secure Context)
</tr>
<tr>
    <td>TISCI_HOST_ID_MAIN_0_R5_3 (38U)
    <td>Cortex R5FSS0_1 (Non-Secure Context)
</tr>
<tr>
    <td>TISCI_HOST_ID_MAIN_1_R5_0 (40U)
    <td>Cortex R5FSS1_0 (Secure Context)
</tr>
<tr>
    <td>TISCI_HOST_ID_MAIN_1_R5_1 (41U)
    <td>Cortex R5FSS1_0 (Non-Secure Context)
</tr>
<tr>
    <td>TISCI_HOST_ID_MAIN_1_R5_2 (42U)
    <td>Cortex R5FSS1_1 (Secure Context)
</tr>
<tr>
    <td>TISCI_HOST_ID_MAIN_1_R5_3 (43U)
    <td>Cortex R5FSS1_1 (Non-Secure Context)
</tr>
\cond SOC_AM64X
<tr>
    <td>TISCI_HOST_ID_A53_0 (10U)
    <td>Cortex A53SS0_0 (Secure Context)
</tr>
<tr>
    <td>TISCI_HOST_ID_A53_0 (11U)
    <td>Cortex A53SS0_0 (Non-Secure Context)
</tr>
<tr>
    <td>TISCI_HOST_ID_A53_0 (12U)
    <td>Cortex A53SS0_1 (Secure Context)
</tr>
<tr>
    <td>TISCI_HOST_ID_A53_0 (13U)
    <td>Cortex A53SS0_1 (Non-Secure Context)
</tr>
\endcond
<tr>
    <td>TISCI_HOST_ID_M4_0 (30U)
    <td>Cortex M4 (Non-Secure Context)
</tr>
<tr>
    <td>TISCI_HOST_ID_ICSSG_0 (50U)
    <td>ICSSG_0 (Non-Secure Context)
</tr>
</table>

- If the resource type required is not present in the array, you need to add this entry and give all the four parameters described above. In this case, make sure that you change the array size accordingly at around line 132 where we specify `resasg_entries_size = ...`. Change that number to reflect the entry added or removed in your case.

\imageStyle{resasg_2.png,width:50%}
\image html resasg_2.png "Resource Assignment Entries Size"

## Rebuilding the board configuration

- Once the changes are made in the file, we need to generate a hex array corresponding to the file and make sure that the bootloader libraries and the `sciclient_set_boardcfg` application are updated with these changes. For this one can follow \ref BOARCFG_GEN section

