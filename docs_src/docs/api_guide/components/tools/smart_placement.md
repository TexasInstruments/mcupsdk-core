# Smart Placement {#SMART_PLACEMENT}

[TOC]

## What is Smart Placement?
TI Sitara MCU have many different levels of memories with varying latency from CPU and peripherals, and while developing an application, it is required to place sections in different memories. Those sections whose destination memory is already known are placed in the targeted memory, however, other sections' destination memory may not be known. For example, in a control loop application, there would an ISRs/functions which would be there in the control path. Since these functions/ISRs are already known to be critical therefore, they are placed in the fastest memory to have maximum performance.

Smart placement is a name given to a process, using which functions and other linker-placed objects are distributed across different memories, while accounting for their criticality, directly from the source code.

## Theory of Smart Placement.

Following table list down memories in ascending order of access latency (not an exhaustive list of memories). Memory with minimum latency is the fastest memory.

\cond SOC_AM243X

Memory Name| Memory Start Address | Memory Size
 ----------|----------------------|---------------
 TCM-A     |0x0                   | 32KB (in split) or 64KB (in lockstep)
 TCM-B     |0x41010000            | 32KB (in split) or 64KB (in lockstep)
 MSRAM     |0x70000000            | 2MB
 FLASH     |0x60000000            | External Flash
\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X

 Memory Name|Memory Start Address | Memory Size
 -----------|---------------------|---------------
 TCM-A      |0x0                  | 32KB (in split) or 64KB (in lockstep)
 TCM-B      |0x8000               | 32KB (in split) or 64KB (in lockstep)
 OCRAM      |0x70000000           | 2MB
 FLASH      |0x60000000           | External Flash
\endcond

From the above table, this device has various memories which can store both code and data. Each memory differs from each other in terms of size and access latency therefore the placement of code and data will determine the performance of code.

### How is identify critical code?

This performance penalty is high in case code or data has to be placed in external memory (or FLASH). In this case, it is beneficial to not place data or code in the external flash that is not critical. Now the criticality of the code can be determined by various factors such as number of times a particular code or data is being accessed or based on the design decision.

Frequency of a function is number of times a function is called. A frequent function would be called a lot of times.

A function's runtime performance can be quantified by `Cycles Per Instruction (CPI)` i.e. average number of CPU cycles spent to execute one instruction of a function. Higher the CPI, sluggish is the runtime performance.

For a function that is called very frequently, usually CPI of that function should be less. Hence, to decide critical functions, two parameters are required to be at hand.

\image html perf_graph.drawio.png

Red region is a region contains all the functions which are frequent and have high CPI. These functions are bottleneck. Green region contains functions which are frequent functions and have low CPI in a given system (without any optimizations). Yellow region contains functions whose performance improvement won't bring any significant improvement as they are not very frequent functions.

Optimization based on profiling should be done such that the result of optimization would bring functions from red regions to green region.

This can be achieved by placing function in red region in faster memory. However, because of limited size of faster memory, not all functions can move to green area. Because of this constraint, functions at top right corner of red region needs to be given more priority over function that is at bottom-left of red region. This priority number will indicative of how critical a function/code is.

### Why CPI of a function increases?

Memory latencies of a memory in which a function is stored is one of the main reason.

Whenever a cache miss happens, instruction/data corresponding to that cache miss would be fetched from a slower memory. If that function is not a cache friendly code, this would lead to high CPI number.

However, sometimes, a badly cached function can still perform better because pre-fetch hardware is able to prefetch most of the instructions. However, these cases are rare and still branch miss-prediction will always increase CPI number.

Therefore, in most cases, increased CPI numbers indicates high cache miss rate as well.

### How smart placement helps in improving performance

#### Providing variety of tools/method to determine critical functions

There are many ways to determine critical function. They are provided and can be read at \ref SMART_PLACEMENT_GETTING_STARTED. Reason of providing this is to make smart placement more usable.

#### Using TCM

TCM is tightly coupled memory and is fastest. This memory is also non-cached. Therefore, after identifying priority critical functions, those functions can be placed in TCM. This would not only improve function's CPI but would also lead to reduced cached miss rate.

#### Removing unnecessary trampolines
\cond SOC_AM243X
\image html smart_placement_am243x_0.png
\endcond
\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
\image html smart_placement_am263x_0.png
\endcond

In above diagram, F1 is a critical function which internally calls F2, F3 and F4. However, because F2, F3 and F4 are placed far away in memory, they cannot be called directly but rather trampolines has be inserted in order to call them. This further reduces the performance. Applying smart placement, not only placed F1 function in faster memory, but also its internally called function, which are also critical function in this case. This also removed trampolines.

### Getting started with Smart Placement

To get started with smart placement, please go through \ref SMART_PLACEMENT_GETTING_STARTED