# Smart Placement {#SMART_PLACEMENT}

[TOC]

## What is Smart Placement?
TI Sitara MCU have many different levels of memories with varying latency from CPU and peripherals, and while developing an application, it is required to place sections in different memories. Those sections whose destination memory is already known are placed in the targeted memory, however, other sections' destination memory may not be known. For example, in a control loop application, there would an ISRs/functions which would be there in the control path. Since these functions/ISRs are already known to be critical therefore, they are placed in the fastest memory to have maximum performance.

Smart placement is a name given to a process, using which functions and other linker-placed objects are distributed across different memories, while accounting for their criticality, directly from the source code.

## When is Smart Placement required?

If code is experiencing the following:
1. Lower then expectation for XIP code performance (only valid for devices that supports XiP)
2. Higher jitter numbers.
3. High Cache miss rate.
4. High CPU loading of ISR/Task.

Then Smart placement can help in diagnosing the above problem.

However this is very application specific, it is expected to yield a better performance if the code is well placed between memories. Smart Placement handles this problem with much ease of use.

## What not to expect from Smart Placement tool?

Depending on the usecase, since, underlying concept of smart placement is to move some function to a faster memory which might help in improving performance with improved ease of use. So in case where a frequently called function or critical function is cached most of the times, then performance improvements using smart placement might not be significant.

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

\cond SOC_AM263X || SOC_AM263PX

 Memory Name|Memory Start Address | Memory Size
 -----------|---------------------|---------------
 TCM-A      |0x0                  | 32KB (in split) or 64KB (in lockstep)
 TCM-B      |0x8000               | 32KB (in split) or 64KB (in lockstep)
 OCRAM      |0x70000000           | 2MB
 FLASH      |0x60000000           | External Flash
\endcond

From the above table, this device has various memories which can store both code and data. Each memory differs from each other in terms of size and access latency therefore the placement of code and data will determine the performance of code.

This performance penalty is high incase code or data has to be placed in external memory (or FLASH). In this case, it is beneficial to not place data or code in the external flash that is not critical. Now the criticality of the code can be determined by various factors such as number of times a particular code or data is being accessed or based on the design decision.

Suppose that for a given program, this criticality has been quantified for each code and data section, i.e. if a program has 2 functions viz. `foo` and `bar` then this criticality is quantified for `foo` to be 1 and for `bar` to be 2. What this tells is, during the placement of these functions, `foo` should be placed in a faster memory than `bar`. Therefore, this criticality has to be translated to placement priority. More critical a function is higher the priority it should have during the placement of that function/data section.

Normally any program has a lot of different functions and data members it accesses and using linker all of those functions and data sections are then distributed across different memories, however, to maximize the performance, it is required to smartly place critical code and data sections in faster memory on priority.

Smart Placement distribute functions across memories based on function's critically.

\cond SOC_AM243X
\image html smart_placement_am243x_0.png
\endcond
\cond SOC_AM263X || SOC_AM263PX
\image html smart_placement_am263x_0.png
\endcond

In above diagram, F1 is a critical function which internally calls F2, F3 and F4. However, because F2, F3 and F4 are placed far away in memory, they cannot be called directly but rather trampolines has be inserted in order to call them. This further reduces the performance. Applying smart placement, not only placed F1 function in faster memory, but also its internally called function, which are also critical function in this case. This also removed trampolines.

Since AM243x/AM263X are Cortex-R5 based SOC, by design TCM-A and TCM-B are not cached and also they are single cycle access memories which makes them the fastest. Other memories, MSRAM/OCRAM are cacheable (however, it depends on MPU settings) but is slower than TCMs and then FLASH is external FLASH and is the slowest.
