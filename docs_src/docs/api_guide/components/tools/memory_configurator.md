#  Memory Configurator {#MEMORY_CONFIGURATOR}

[TOC]

## Introduction

In the prior versions of the SDK, linker files presented challenges in terms of ease of maintenance. Linker files were generated from templates, meaning that any alterations to parameters such as stack size, heap size, memory regions, or sections necessitated direct edits to the linker files. This process proved to be cumbersome and difficult to manage.

The Memory Configurator tool seamlessly integrated into sysconfig empowers users to configure critical parameters. By utilizing this tool, corresponding linker files are automatically generated. This feature significantly streamlines the process, eliminating the need for manual adjustments to the linker files and providing users with a hassle-free experience.


## Enhanced Significance in Multi-Core Environments

\imageStyle{multi_core.png,width:50%}
    \image html multi_core.png "Demonstrating a multi core scenario"

Suppose core 0 has a memory region named MEM0 with a start address of 0x0 and a length of 16KB. Core 1, on the other hand, possesses a memory region named MEM1 with a start address of 0x4000 and a length of 64 KB. Now, in the event that the user wishes to expand the size of MEM0, this action would necessitate an adjustment in the start address of MEM1 while ensuring its size remains unchanged. Furthermore, if there are any other memory regions defined immediately after MEM1, all corresponding adjustments would need to be made. The integration of this tool facilitates these modifications, sparing the user the need to manually edit each individual configuration.

## Handling Violations and Common Configuration Errors

There are scenarios where a core attempts to configure the Memory Protection Unit (MPU) for a memory region that hasn't been assigned to it. This constitutes a violation that requires proper handling.

Another common error involves designating a shared memory region as cached. This tool serves as a safeguard against such mistakes, ensuring that these violations and configuration errors are prevented.

## Getting Started with the Tool

Within the tool interface, the user will find a section labeled "MEMORY" on the left pane. This section encompasses three key components: "General," "Memory Region," and "Sections."

In the forthcoming discussion, each of these components will be explored in detail. This will equip users with the knowledge and guidance required to configure the tool in alignment with their specific requirements.

### General

Within this section, the user will have the option to configure essential parameters such as stack size, heap size, IRQ (Interrupt Request), FIQ (Fast Interrupt Request), SVC (Supervisor Call), Abort Stack and Undefined Stack sizes.

Upon adding an instance, these parameters will be automatically populated with their default values.

Additionally, the user will have the capability to select the compiler, which will determine the syntax for generating the linker files.

\imageStyle{general.png,width:90%}
    \image html general.png "Sysconfig view of General and the corresponding Linker File generation"

### Memory Region

\imageStyle{memory_region_start.png,width:50%}
    \image html memory_region_start.png "Initial Sysconfig view of Memory Region"

The Memory Region configuration is a pivotal aspect with several integrated features.

Upon clicking the "Add" button, an option titled "Automate MPU Setting" will become available. By default, this option will remain unchecked. In this scenario, users can add memory regions without any associated MPU settings, unless configured otherwise. There is no restriction on the size in the sense that it doesn't need to be power of 2 and can take any value within its memory type limit.

If the user opts to check this option, it signifies that each memory region instance may be linked to a maximum of one MPU configuration. This implies that when a user adds a memory region instance and desires to include an MPU setting, they are exempted from the manual entry of parameters such as address and size.

The tool employs a specific algorithm for memory region allocation:

1. It initially identifies all memory region instances of a particular type.

2. Instances with manually assigned start addresses take precedence and are positioned at the forefront within the memory region of that type.

3. Subsequently, any remaining unassigned regions result in empty spaces, known as "holes." These holes are then sorted from smallest to largest.

4. For memory regions with automatically calculated start addresses, the tool allocates them to the smallest hole that has sufficient space to accommodate them.

5. In the event that the tool encounters size constraints preventing the allocation of a memory region, it will promptly flag an error.

Here is an illustration to explain the above:

\imageStyle{memory_region_eg_ocram1.png,width:50%}
    \image html memory_region_eg_ocram1.png "Memory Region Configuration for OCRAM1"

\imageStyle{memory_region_eg_ocram2.png,width:50%}
\image html memory_region_eg_ocram2.png "Memory Region Configuration for OCRAM2"

\imageStyle{memory_region_eg_ocram3.png,width:50%}
    \image html memory_region_eg_ocram3.png "Memory Region Configuration for OCRAM3"

\imageStyle{memory_region_eg_ocram4.png,width:50%}
    \image html memory_region_eg_ocram4.png "Memory Region Configuration for OCRAM4"


In this configuration, there are four designated regions: OCRAM_1 and OCRAM_2 have auto-generated start addresses, while OCRAM_3 and OCRAM_4 possess manually assigned ones.
Following the algorithm's guidelines, the regions with manual start addresses, OCRAM_3 and OCRAM_4, take precedence. They are first accommodated within the OCRAM space.
Subsequently, three gaps remain: one to the left of OCRAM_3, another between OCRAM_3 and OCRAM_4, and a third to the right of OCRAM_4.
The algorithm then selects the smallest available gap to house the largest auto start address region, OCRAM_2.

Afterwards, OCRAM_1 is placed in the gap adjacent to OCRAM_2, as it is the smallest one capable of accommodating OCRAM_1.

\imageStyle{memory_region_eg_layout.png,width:90%}
    \image html memory_region_eg_layout.png "Memory Region Configuration Layout"

\note
Once a memory region is assigned to a core, it is prohibited from being overlapped by any other region, whether by the same core or a different one. For memory types like TCMA and TCMB, which are core-specific and distinct at the physical level, it is permissible to use the same start address and size. This is because, at the physical level, it does not constitute actual overlap.


#### Shared Memory Region Configuration

It's important to note that if a memory region is designated as "Shared," the user will have the option to select the cores with which it will be shared. As a result, this region will automatically be included in the linker files of all the chosen core. Therefore, there is no need for the user to manually add the same region to the other cores. In fact, doing so would not be possible. For shared memory regions, it's essential to ensure that the size is a power of 2 else an error will be triggered. Also, the start address has to be aligned with the size of the memory region.

\note
The same shared region instances will not appear in the GUI for the selected cores.

#### Impact of Automate MPU Setting

When the "Automate MPU Setting" option is selected, the tool's behavior is adjusted to align with MPU setting rules. Specifically, memory regions cannot be placed anywhere within a hole; they must be situated at an address that starts at a multiple of their size. The size must be a power of 2 starting from 32 B through 4 GB.

MEM0 and MEM1, both 32B regions of the same type, are arranged as depicted. However, MEM2, which is 128B in size, cannot be positioned immediately after MEM1 due to alignment constraints. Placing it there would result in a start address that is not a multiple of its size. Consequently, the region is relocated to an address where the start aligns with its size, thereby generating a gap between MEM1 and MEM2.

\imageStyle{memory_region_start_address_alignment.png,width:90%}
    \image html memory_region_start_address_alignment.png "Memory Region Start Address Alignment"

An MPU setting will be generated with parameters (such as start address and size) mirroring those of its associated memory region. Please note that these parameters will be locked and cannot be altered by the user.
Other parameters, such as Access Permissions, Region Attributes, Allow Code Execution, and Sub-Region Disable Mark, can be customized by the user.

\imageStyle{memory_region_mpu.png,width:90%}
    \image html memory_region_mpu.png "Sysconfig view of Memory Region with Automate MPU Setting"

### Section

Within this section, the user has the option to set various parameters for a section.

1. Load Memory and Run Memory Options: These will list all available memory regions added by the user in the previous "Memory Region". By default, the run memory will match the load memory, unless the user chooses to make a specific change.

\imageStyle{section_load_memory.png,width:60%}
    \image html section_load_memory.png "Sysconfig view of Section: Load memory options"

2. Section Type Selection: The user can choose from options like LOAD, DSECT, COPY, NOLOAD, and NOINT.

3. Output Section Requirement: It is mandatory for the user to add at least one output section.

4. Output Section Grouping: The user can opt to either group all output sections together or keep them separate. The configurations for load and run memory, as well as section type, will apply to all output sections under a given instance.

5. Input Section Addition: The user also has the flexibility to add zero or more input sections under a specific output section.

\imageStyle{section.png,width:90%}
    \image html section.png "Sysconfig view of Section and the corresponding Linker File generation"


### MPU Settings

As discussed earlier, let's revisit the functionality of MPU settings with the Automate option.

When the user selects the "Automate MPU Settings" option within the Memory Region, they will have the opportunity to configure the MPU settings specific to that memory region. Note that in this scenario, the start and size parameters are fixed and cannot be altered. This enhancement aims to streamline the configuration process, alleviating the need for separate MPU adjustments.

However, it's worth mentioning that there may be instances where users prefer the flexibility to customize MPU settings. In such cases, they have the option to disable the automation feature.

\note
Even with the automate option enabled, users still retain the ability to add custom MPU configurations with specific addresses or sizes, if necessary. On the other hand, when the automation is disabled, users will be required to manually add all MPU configurations.

When "Automate MPU Setting" is not selected within the memory region, it results in the loss of the direct one-to-one mapping of the memory region to the MPU configuration view. In such cases, it is beneficial to provide users with a comprehensive overview of how the region is composed.

This should encompass the following information:

1. Constituent Regions: Display all the individual regions added within the memory region section that collectively constitute this particular region.

2. Shared Region Information: Specify if any portion of this region is shared among multiple cores. Additionally, indicate which specific cores have access to this shared region.

3. Memory Type: Clearly state the type of memory being utilized (e.g., Flash, TCMA, etc.) for this particular region.

\imageStyle{mpu_settings.png,width:90%}
    \image html mpu_settings.png "Sysconfig view of MPU configuration integrated with Memory Region attributes"

This approach ensures that users have a clear understanding of how the manually configured MPU settings relate to the broader memory regions and their usage across different cores.

### Enhanced Visualization View

To facilitate user understanding and ease of navigation, a specialized view has been incorporated. This view provides a visual representation of the memory type, displaying crucial details such as start address, size, accessibility across cores, and direct links to the respective memory regions for seamless navigation.

\imageStyle{memory_region_summary.png,width:70%}
    \image html memory_region_summary.png "Consolidated view of Memory Regions"

A similar visual representation has been integrated for MPU configurations. This view presents a detailed tabulation of all configurations, providing users with a clear overview of their settings.

\imageStyle{mpu_setting_summary.png,width:70%}
    \image html mpu_setting_summary.png "Consolidated view of MPU Settings"