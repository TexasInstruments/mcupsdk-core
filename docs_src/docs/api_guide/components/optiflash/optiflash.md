# OptiFlash Memory Technology {#OPTIFLASH}

[TOC]

## Introduction

High performance microcontrollers differ in architecture from traditional microcontrollers. Following diagram shows the differences.

\imageStyle{architecture_of_high_performance_mcu.png,width:50%}
\image html architecture_of_high_performance_mcu.png "Difference in architecture of a traditional Microcontroller and a typical high-performance microcontroller like AM2x. Note that, there is no embedded flash."

New application software in automotive industry like networking, zonal etc. that runs on these high-performance microcontrollers, requires more than 1 number of CPU, in range of 2-10, and CPU running at core frequencies of 200MHz to 1GHz.

However, in terms of manufacturing, it requires advanced CMOS process nodes and embedded flash is not possible. Therefore, the architecture of these high performance MCU is such that there is no embedded flash and this is very new architecture for microcontroller market.

However, having an external flash architecture comes with its own advantages such as the cost advantage and is estimated of range 20cents to $2 based on flash size.

The other advantage that this gives is memory scalability i.e. to same port, 8MB flash to 128MB flash or more can be attached.

Although having an external flash provides all these advantages, it also comes with its own challenges which needs to be overcome to make external flash microcontroller flexible and robust enough to be used in variety of use case.

### Challenges with executing from external flash

Since external flash is connected to microcontroller via pins via tracks on PCB board, there are some serious challenges. First off, number of pins with which external flash is connected is limited therefore, the data bus width is also limited. This limits the performance of XIP and data transfer between external flash and CPU. However, increasing the data bus bits would translate making use of a greater number of pins which itself is a limited resource. Increasing the clock frequency would increase the complexity of board design. Other than this, communication protocol also has an impact on performance. Hence, performance of external flash is limited by a number of parameters and is generally slower than an embedded flash and this concern needs to be addressed.

To make execution from external flash safe and secure, some additional functionality is required. However, these necessary functions also have an impact on performance.

## OptiFlash Technology

OptiFlash technology is an ecosystem of patented hardware IP, software tools and drivers which aims to at least overcomes all of the previously described generic challenges.

The following diagram shows the OptiFlash technology ecosystem.

\imageStyle{optiflash_entire.png,width:70%}
\image html optiflash_entire.png "OptiFlash technology Ecosystem"

AM263Px is 1st device that comes with OptiFlash technology and is next in series with AM263x (AM263x does not have OptiFlash technology).

Following is a representative diagram of AM263x device which does not have OptiFlash technology.

\imageStyle{am263x_hardware_arch.png,width:50%}
\image html am263x_hardware_arch.png "Communicating with external flash in AM263x device"

Here, external flash is communicated over QSPI. Any data read/write is being forwarded to QSPI IP.

Coming text will explain how OptiFlash technology is addressing all the previous concerns and new features that are introduced with it.

### XIP Performance.

With OptiFlash technology the following hardware blocks has been introduced for XIP performance.

\imageStyle{am263px_perf.png,width:50%}
\image html am263px_perf.png "AM2x devices with OptiFlash technology. Blocks marked in RED are specifically put to improve XIP performance."

#### OSPI

QSPI has been replaced with OSPI or Octal-SPI. The main difference is in the bus width and clock frequency. In AM263Px, OSPI is running at max 133MHz DDR on 8 data lines.

#### RL2

RL2 or remote Layer 2 cache is added. This, basically, is a L2 cache which can be configured to have different size and its cache bank can also be configured to any place in L2 memory.

What this allows is to have a L1 and L2 cache architecture in microcontroller. L1 cache being the CPU cache and L2 being RL2 cache. One point to note is that since external flash only contains read-only data, L2 cache is effectively caching code and rodata only.

#### FLC For Overlays

FLC or Fast Local Copy is essentially a very simplified DMA but with the ability to allow concurrent execution.

\imageStyle{flc_lookup_function.png,width:50%}
\image html flc_lookup_function.png "FLC HWA or FLC hardware Accelerator, would fetch an instruction from faster internal memory, if available, else would get it from external Flash."

FLC power is in simplicity of programming and Concurrent copy & execution. Only 3 registers are required to be programmed viz. source address, size of transfer and destination address.

Above diagram show the working of FLC. When CPU request an address, if FLC sees that a particular address is copied in the internal SRAM then the data corresponding to that address is returned from SRAM else it is fetched from External Flash. Because the data is returned from internal RAM than external FLASH, fetch time is reduced and this provides some boost in performance.

Although use case of FLC are many, but, from the perspective of system performance improvement, FLC can also be used for implementing **overlaying** schemes.

#### Smart Placement

Smart placement is a name given to a process, using which functions and other linker-placed objects are distributed across different memories, while accounting for their criticality, directly from the source code with an intention to improve run-time performance. More on this can be read at \ref SMART_PLACEMENT.

#### Using different blocks for Performance Improvement.

All the above different blocks can be used together to boost the system level performance and the way that can be achieved is
1. Configure OSPI to work in highest speed configuration.
2. Use Smart Placement to identify critical functions and place them in faster memory.
3. Use Overlay schemes to download function at runtime.
4. Enable RL2


The Logic behind these steps is, suppose, all the functions in a program are in external FLASH and are being XIPed and to improve system performance, it is required to bring a few functions in the internal memory because internal memory is limited. Now the question is what functions are to be brought in the internal memory? Answer to this question is smart placement. Using smart placement, all the performance critical function can be brought in the internal memory (which is fast). 

However, there can functions which are performance critical but couldn't be placed in internal memory (lets say the internal memory is full). These functions can be accelerated using runtime function overlays. So such critical function's performance is improved by bringing them in internal memory.

For rest of the functions which are either not critical or were impossible to be accommodated in the internal memory, then can be accelerated by using L2 cache or RL2.

Notice that, the effort here is place critical function in the faster memory and all these tools are used to achieve this. 

Enabling RL2 is the easiest and quickest step. So, enabling it in during the starting of the development is also possible.


### Safety & Security

\note Enabling security is under development. 

\imageStyle{am263px_saf_sec_perf.png,width:50%}
\image html am263px_saf_sec_perf.png "AM2x devices with OptiFlash technology. Blocks marked in RED are specifically put to provide safety and security."

For safety and security, hardware blocks have been provided. These blocks provide on-the-fly safety and security by splitting up the data in external flash into 32bytes chunk and for each chunk, ECC (Error Correction Code) and MAC (Message Authentication Code) is added as header and footer to that data chunk which is also called metadata.

To enable Safety and Security, all that is required is to enable the hardware. However, it is required to be made sure that the data that is stored in the external flash is also encrypted so as to be read by this safety and security module.

This safety and security module is called ECCM and OTFA, respectively.

The way this hardware works is that when CPU requests to fetch an instruction, the entire chunk with its metadata, corresponding to that instruction, is fetched by OSPI controller which is then passed to the ECCM and OTFA hardware which then decrypts and verify data, on-they-fly, and then the instruction is passed on to the CPU to execute.

This allows XIP with full safety and security using hardware accelerators. Using hardware accelerators reduces the impact on performance.

\note for more information on how to enable safety, please refer to \ref OPTIFLASH_ECCM

### OptiShare: Removing Redundant Code

For example implementing optishare see: \ref EXAMPLES_DRIVERS_IPC_NOTIFY_ECHO_OPTISHARE

For indepth knowledge of OptiShare see: \ref OPTIFLASH_OPTISHARE

When it comes to high performance MCU which has many cores, using traditional method of compilation is not efficient. Normally, for a microcontroller, compilation goes like the follows:
1. Compilation/assembling of source code written in different programming languages.
2. Linking compiled/assembled code and statically linking libraries.

However, this way of compilation does not scale well. Because if all core's application is using same library, for ex. FreeRTOS or LwIP, etc. then those libraries will be sitting 4 number of times if 4 core's application is using same library.

To address this issue, if all the libraries are placed in a shared memory and then all cores calls these functions from shared memory.

This would change the compilation process to the following:

1. Compilation/assembling of source code written in different programming languages.
2. Linking compiled/assembled code and statically linking libraries.
3. Identify common code in binaries of all core.
4. Relink all the core's binary by removing the common code from each core's binary.
5. Create a separate binary which only contains shared binary.

To enable this sharing of code, OptiFlash technology provides hardware and software tools solution. Software tool is called OptiShare.

During compilation of multi-core project, OptiShare is responsible to identify common code and separation of it from rest of all core's binaries.

\imageStyle{optishare_offline.png,width:50%}
\image html optishare_offline.png "OptiShare with remove redundant code/data and will put it once in Shared Object"

However, this way of separating common code does not work standalone. Special hardware needs to be programmed, for this shared code scheme needs to work.

This special hardware that is introduced with OptiFlash technology is called RAT or Region Address Translation.

\imageStyle{optiflash_rat_safe_sec_perf.png,width:50%}
\image html optiflash_rat_safe_sec_perf.png "To address code/data redundancy, RAT hardware is added as part of OptiFlash technology"

At run time, this hardware needs to be programmed by the application software before any shared code be called, otherwise, CPU Abort Exceptions will occur.

\imageStyle{optishare_online.png,width:50%}
\image html optishare_online.png "At runtime, RAT is programmed, and this maps the CPU specific data to shared code."

### How to perform Firmware-Upgrade-Over-Air

In typical microcontroller with embedded flash, to be able to perform FOTA, embedded flash size should be more than the double the size of application, because, at a time 2 copy of same application would be stored, in flash. One would be an active application and other would be different version of the application. However, this puts high constraint on embedded flash requirement and this requirement translates to increased BOM cost. Effectively, for an active application, available internal memory is half of total internal memory.

With external flash solution with OptiFlash technology technology, entire internal memory can be used for active image of application. For example, in AM263Px, internal RAM is 3.5MB of total internal memory. An active image of application can use entire internal memory and for new version image of application can be stored in external Flash. Therefore, when looking from typical microcontroller perspective, effective total memory available in AM263Px is alteast 7MB.

However, because OptiFlash technology technology enables XIP, FOTA is difficult to be performed. External flash is connected to microcontroller over 8 data lines (OSPI) and on same data lines, both reads and writes would happen. And in FOTA, writes would be happening while XIP is going on. During writing, reading should be stopped which also mean that XIP needs to be stopped and this would have a huge impact on performance and increased complexity in system design.

Although, FOTA is usually a very low priority task and XIP downtime can be managed in most situation. However, in case, XIP downtime is coming out to be critical, then RWW flash can be used with OptiFlash technology FOTA hardware accelerator.

\imageStyle{optiflash_fota_rat_safe_sec_perf.png,width:50%}
\image html optiflash_fota_rat_safe_sec_perf.png "FOTA hardware accelerator is added as part of OptiFlash technology to schedule reads and writes so as to minimize XIP downtime."

This role of this accelerator is to schedule reads and writes over same 8 data lines of OSPI. The aim of this hardware is to minimize XIP downtime.

To read more on this please refer to 
1. \ref FLSOPSKD_IP
2. \ref bootseg_ip_working

## XIP Performance Benchmarks

### Benchmark 1

When on same application (Benchmark #1), Smart Placement is applied, then 10% boost in performance is seen. More on this can be read at \ref BENCHMARK_SMART_PLACEMENT.
