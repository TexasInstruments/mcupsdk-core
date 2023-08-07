# Optimizing applications with MCU SDK {#OPTIMIZING_APPLICATIONS_WITH_MCU_SDK}

[TOC]


## Optimizations {#CHAPTER_OPTIMIZATION}

Here is a short summary of the performance optimizations discussed in detail in this document.

- \ref CHAPTER_OPTIMIZATION_SECTION_1
- \ref CHAPTER_OPTIMIZATION_SECTION_2
- \ref CHAPTER_OPTIMIZATION_SECTION_3
- \ref CHAPTER_OPTIMIZATION_SECTION_4
- \ref CHAPTER_OPTIMIZATION_SECTION_5

Memory optimizations

- \ref CHAPTER_OPTIMIZATION_SECTION_6

### Optimize Interrupt handling (IRQ/FIQ) {#CHAPTER_OPTIMIZATION_SECTION_1}

\note Following optimizations are specific to ARM R5 architecture.

In MCU SDK Driver Porting Layer (DPL), the NORTOS IRQ handler for R5F uses a common dispatcher (HwiP_irq_handler and HwiP_irq_handler_c) for handling interrupt, to look-up and call user specified ISR function.
This has relatively high interrupt latency.

The optimized approach makes use of dedicated IRQ handlers per interrupt which then calls the user specified ISR function. This has a reduced interrupt latency.


#### Understand interrupt and exception handling of the ARM Cortex-R5 processor

This section describes the interrupt and exception handling of the ARM Cortex-R5 processor as implemented on Sitara MCU microcontrollers, as well as the related operating modes of the
processor.

Exceptions are interruptions of the normal program flow this includes peripheral interrupts. 
The Cortex-R5 processor usually takes care to preserve the critical parts of the current processor state, so that the normal program flow could be resumed after the exception was handled by the application (saving and restoring of the CPSR and banked Stack Pointers).
The Cortex-R5 processor implements the following exceptions:

- SuperVisor Call (SVC) also known as Software Interrupt (SWI)
- Undefined Instruction (UNDEF)
- Prefetch Abort (PABT)
- Data Abort (PABT)
- Fast Interrupt Request (FIQ)
- Interrupt Request (IRQ)
- Reset
- Break Point (BKPT)

Interrupt Request (IRQ)

The nIRQ is an input to the processor core, a low signal on the nIRQ input causes the processor to take the IRQ exception, if not masked. 
On the Sitara MCU’s the nIRQ is connected to the VIM. 
It is usually used as “general-purpose” interrupt line and interrupt dispatching is usually handled by the VIM. 
The Cortex-R5 processor offers a so called VIC port to supply the interrupt vector address directly to the processor, in order to reduce interrupt latency for IRQ’s. 


Vectored Interrupt Manager (VIM) Module

The Vectored Interrupt Manager (VIM) module provides hardware assistance for prioritizing and controlling the many interrupt sources present on Sitara family devices. 
Interrupts are caused by events outside of the normal flow of program execution. 
Normally, these events require a timely response from the central processing unit (CPU); therefore, when an interrupt occurs, the CPU switches execution from the normal program flow to an ISR.

The VIM module has the following features:
- Currently supports up to 256 interrupt channels
 - Provides programmable priority and masks for interrupt request lines
- Provides a direct hardware dispatch mechanism for fastest IRQ dispatch via the VIC port (Hardware Vectored Interrupts)


Hardware Vectored Interrupts (only IRQ)

1. Request IRQ only.
2. CPU reads IRQ vector address instead of 0x18.
3. VIM provides address of highest pending request directly to the processors VIC port.
4. CPU branches directly to ISR.

\imageStyle{Fig17.PNG,width:80%}
 \image html Fig17.PNG "Hardware Vectored Interrupt Connection Scheme in ARM Cortex R5F in AM263x"


#### Understanding NORTOS IRQ handler in MCU PLUS SDK DPL

The ARM Cortex-R5 (ARMv7-R architecture) processor does not support interrupt nesting in hardware, as some Cortex-M (ARMv7-M architecture) processors do. 
Only a two level nesting is possible when using IRQ and FIQ, where the FIQ can interrupt the IRQ.

\note In MCU PLUS SDK , IRQ nesting is emulated in software with the help of reentrant interrupt handlers.

This is mainly because the core has only one Saved Program Status Register (SPSR) and one Link Register (LR) register for the IRQ mode.
If an IRQ is interrupted by another IRQ, these CPU registers would get overwritten (corrupted) and a later restoring of the processor state would not be possible. 
Also, the nature of the ARM C implementation has to be taken into account, so that non-callee-saved registers (R0-R3 and R12) have to be preserved between function calls. 
To work around these limitations, the CPU registers (SPSR, LR, R0-R3 and R12) have to be preserved on the stack, by an ISR handler. 
Furthermore, the CPU mode has to be switched to another mode, usually System mode, as the IRQ has to be re-enabled, which causes the current LR (used by subroutines) to be overwritten when the CPU is still in IRQ mode. 

As the ISR is executed in User or System mode and not in IRQ mode the “main” stack is used and not the IRQ exclusive stack. 
Usually the System mode is used, as this offers privileged access similar to the IRQ mode. 
Before the interrupts could be enabled again, it has to be ensured that the current interrupt source is cleared or masked, so that the ISR is not immediately interrupted by “itself”.
ARM suggests a flow to implement a reentrant IRQ handler, which is described below. 
However, this flow should be optimized to get the shortest interrupt latency and has to be extended by special Vectored Interrupt Manager (VIM) handling to work on Sitara MCUs. 
This flow is described in detail below

Suggested Flow by ARM

1. Construct the return address and save it on the IRQ stack.
2. Save the work registers, non callee-saved registers and IRQ mode SPSR.
3. Clear the source of the interrupt.
4. Switch to System mode, keeping IRQs disabled.
5. Check that the stack is eight-byte aligned and adjust, if necessary.
6. Save the User mode LR and the adjustment, 0 or 4 for architectures v4 or v5TE, used on the User mode SP.
7. Enable interrupts and call the C interrupt handler function.
8. When the C interrupt handler returns, disable interrupts.
9. Restore the User mode LR and the stack adjustment value.
10. Readjust the stack, if necessary.
11. Switch to IRQ mode.
12. Restore other registers and IRQ mode SPSR.
13. Return from the IRQ.

Modified IRQ Handler Flow to Work With VIM

\imageStyle{Fig13.PNG,width:80%}
 \image html Fig13.PNG "ARM Cortex R5 interrupt handling in MCU PLUS SDK for AM263x"

The OS abstraction layer (DPL) of MCU PLUS SDK architected for networking applications introduces performance overheads (interrupt latencies). To support Real Time Control use cases the IRQ handlers are optimized, reducing software overheads down to 150ns. Use below steps to utilize the optimized IRQ handlers.

#### Step by step guide for optimizations

\note Please refer \ref EXAMPLES_KERNEL_DPL_LOW_LATENCY_INTERRUPT example which demonstrates the steps to register a custom interrupt using optimized approach

Steps to register a custom interrupt using MCU SDK (default approach)

\imageStyle{Fig14.PNG,width:80%}
 \image html Fig14.PNG "Steps for MCUSDK default"

Steps to register a custom interrupt using optimized approach

\imageStyle{Fig15.PNG,width:80%}
 \image html Fig15.PNG "Steps for optimized IRQ handler registration"

Choose IRQ handler macro based on application need:

\imageStyle{Fig16.PNG,width:80%}
 \image html Fig16.PNG "IRQ handler macros - functionality and latency"


For FIQ, use the default approach in MCU SDK. The Fast Interrupt operating mode has eight processor registers banked (R8 - R12, the SP, LR and the SPSR) and has the advantage that these registers do not have to be preserved or saved to the stack in order to use them in an interrupt handler. This improves interrupt latency for FIQ. In the case only one interrupt is mapped to the FIQ, the whole interrupt service handler could be placed at this address (0x1C) to further improve interrupt latency (avoiding unnecessary branches).



### Optimize memory placement {#CHAPTER_OPTIMIZATION_SECTION_2}


\note For optimal performance, update linker command file to place control ISR code and data in TCM (Tightly Coupled Memory) for best execution performance (1 cycle read write access).


Placement of interrupt handler (which invokes the user defined ISR function), ISR function and data in memory (TCM - Tightly coupled memory or OCRAM - On chip RAM) is a crucial factor

Check the linker command file, compiler attributes of the relevant functions, assembly listing filie and generated .map file to find out where it is placed

Function: 
HwiP_irq_handler
(assembly function defined in HwiP_armv7r_handlers_nortos_asm.S)

Snippet:
\code
/* FUNCTION DEF: void HwiP_irq_handler(void) */
        .global HwiP_irq_handler
        .type HwiP_irq_handler,%function
        .section ".text.hwi","ax",%progbits
        .arm
        .align 2
\endcode

As shown, this function is part of .text.hwi section

In the linker command file (linker.cmd), check the MEMORY for this SECTION

For example, in below snippet from linker.cmd file the .text.hwi is placed in TCM A

\code
	GROUP {
		.text.hwi: palign(8)
		.text.cache: palign(8)
		.text.mpu: palign(8)
		.text.boot: palign(8)
		.text:abort: palign(8) /* this helps in loading symbols when using XIP mode */
	} > R5F_TCMA 
\endcode


\note Refer this link for info on how to capture and use core trace data to visualize code profiling and coverage data in CCS. https://www.youtube.com/watch?v=4hEY0sZToUE


### Optimize R5F MPU settings {#CHAPTER_OPTIMIZATION_SECTION_3}

\note  Recommended R5 MPU settings for control peripherals: Device, Non-Shareable. 
\note  MPU settings for On-Chip RAM shall not be kept shareable unless specifically intended shared region for IPC. 

Modify R5F MPU configuration for the ControlSS register space to enable posted writes.

\imageStyle{Fig11.PNG,width:80%}
 \image html Fig11.PNG "MPU settings for EPWM periperals"


#### Background info on MPU and Cache settings

This section provides an overview of various features available in R5F MPU.
Cortex-R5 has an in-built Memory Protection Unit (MPU) module that helps configure the memory types and attributes as defined in a processor’s memory ordering mode. 
The MPU is specific to each core in the system and can only modify the memory ordering model of the CPU to which it is attached.


\imageStyle{Fig12.PNG,width:80%}
 \image html Fig12.PNG "MPU settings"


ARM Cortex-R5 MPU Settings

Memory Types
- Strongly Ordered
 - All memory accesses to Strongly Ordered memory occur in the program order.
 - An access to memory marked as Strongly Ordered acts as a memory barrier to all other explicit accesses from that processor, until the point at which the access is complete
 - All Strongly Ordered accesses are assumed to be shared
 - It is recommended to configure the external peripheral or FIFO logic (accessible via EMIF) as strongly ordered
- Device
 - Defined for memory locations where an access to the location can cause side effects
 - The load or store instruction to or from a Device memory always generates AXI transactions of the same size as implied by the instruction
 - Can be shared or non-shared
 - It is recommended to configure the peripheral register spaces as device type
- Normal
 - Defined for memories that store information without side-effects. For example, RAM, Flash
 - Can be shared or non-shared
 - Can be cached or non-cached

In case there are multiple accesses to a normal memory, the CPU might optimize them leading to a different set of accesses of different size or number. 
The order of accesses may also be altered by the CPU. 
CPU makes an assumption that the order or number of accesses to a normal memory is not significant. 
For example, two 16-bit accesses to consecutive normal memories may be combined to a single 32-bit access. 
Whereas, in case of device and strongly-ordered memories, the CPU always performs the accesses in the order specified by the instructions. 
CPU does not alter the order, size or number of accesses to these memories. 
Device accesses are only ordered with respect to other device accesses, while strongly ordered memory accesses are ordered with respect to all other explicit accesses.
It is to be noted that strongly-ordered memory leads to a larger performance penalty.

Shared and Non-Shared Memories

Shared memory attribute permits normal memory access by multiple processors or other system masters whereas non-shared memories can only be accessed by the host CPU.
The processor’s L1 cache does not cache shared normal regions. This means that a region marked as shared is always a non-cached region (this device does not support L2 cache).

Cache Settings

This device only supports L1 cache. Cache property is only applicable for normal memories. 
Due to the unavailability of L2 cache, cache is applicable only for normal non-shared memories.
The following are various configurations available for cache:
- WTNOWA - Write-Through, No Write-Allocate
- WBNOWA - Write-Back, No Write-Allocate
- WBWA - Write-Back, Write-Allocate
If an access to a cached, non-shared normal memory is performed, cache controller does a lookup in the cache table. 
If the location is already present in the cache, that is a cache hit, the data is read from or written to the cache. 
If the location is not present, that is a cache miss, it allocates a cache line for the memory location. 
That means, the cache is always Read-Allocate (RA). In addition, data cache can allocate on a write-access, if the memory is marked as Write-Allocate (WA). 
Write accesses that are cache-hit, are always written to the cache locations. 
If the memory is marked as Write-Through (WT), the write is performed in the actual memory as well. 
If the memory is marked as Write-Back (WB), the cache line is marked as dirty, and the write is only performed on the actual memory when the line is evicted.

MPU Regions

The MPU present in this device supports up to 16 regions. 
The memories accessed by the CPU can be partitioned up-to 16 regions (with region 0 having the lowest priority and region 15 having the highest). 
Each can be configured to a specific memory type and assign required permissions.
When the CPU performs a memory access, the MPU compares the memory address with the programmed memory regions. 
If a matching memory region is found, it checks whether the required permissions are set. 
If not, it signals a Permissions Fault memory abort. 
If the matching memory region is not found, the access is mapped onto a background region. 
If background region is not enabled, it signals a Background Fault memory abort.


MPU settings can be easily done using Sysconfig.

Recommended MPU settings for the memories:

Flash
- Accessed only by the CPU
- Can be split into Privileged and non-privileged regions. Typically, in an RTOS context, the tasks are
executed in the user mode and the kernel code is executed in the privileged mode. The tasks can
be placed in the non-privileged flash section and kernel code can be placed in privileged section to
ensure that no user task accidentally executes the kernel functions.
- Privileged Flash Region
 - Type : Normal, Non-Shareable, Cacheable
 - Permission : Privileged Read Only, Executable 
- Non-privileged Flash Region
 - Type : Normal, Non-Shareable, Cacheable
 - Permission : Privileged/User Read Only, Executable

RAM
- Can be accessed by CPU and other bus masters
- Can be split into shared and non-shared regions
 -  Shared RAM (accessed by other bus master master like DMA or EMAC)
  - Type : Normal, Shareable, Non-Cacheable
 - Non-Shared RAM (accessed only by the CPU)
  - Type : Normal, Non-Shareable, Cacheable
  - Use WriteBack mode for faster accesses
  - Use WriteThrough mode if any of the other masters does a read to this memory
- Can be split into privileged and non-privileged regions. Privileged RAM are typically used to store data accessible from a privileged code
 - Privileged RAM
  - Permission : Privileged Read-Wriite, Non-Executable
 - Non-Privileged RAM
  - Permission : Privileged/User Read-Wriite, Non-Executable
- RAM can also be used to store the code to which CPU can branch and execute
 - Executable RAM
  - Permission : Privileged (or Privileged/User) Read-Write, Executable

Peripherals
- Accessed only by CPU
- Type : Device, Non-Shareable
- Permission : Privileged/User Read-Write, Non-Executable

External memories (accessed via EMIF module)
- External SDRAM (accessed only by CPU)
 - Type : Normal, Cacheble, Non-Executable/Executable
- External peripherals or FIFO memories
 - Type: Strongly-Ordered





### Optimize compiler settings {#CHAPTER_OPTIMIZATION_SECTION_4}

\note Recommended optimization level to use with MCU SDK: -Os. This option gives balance between code size and performance.
Please note -O3 option is recommended for optimizing performance, but it is likely to increase compiler generated code size.
Refer https://software-dl.ti.com/codegen/docs/tiarmclang/rel2_1_0_LTS/compiler_manual/using_compiler/compiler_options/optimization_options.html

Choose compiler code generation settings (optimization levels O[0|1|2|3|fast|g|s|z]) according to application need.

The tiarmclang compiler supports a variety of different optimization options, including:

- -O0 - no optimization; generates code that is debug-friendly
- -O1 - restricted optimizations, providing a good trade-off between code size and debuggability
- -O2 or -O - most optimizations enabled with an eye towards preserving a reasonable compile-time
- -O3 - in addition to optimizations available at -O2, -O3 enables optimizations that take longer to perform, trading an increase in compile-time for potential performance improvements
- -Ofast - enables all optimizations from -O3 along with other aggressive optimizations that may realize additional performance gains, but also may violate strict compliance with language standards
- -Os - enables all optimizations from -O2 plus some additional optimizations intended to reduce code size while mitigating negative effects on performance
- -Oz - enables all optimizations from -Os plus additional optimizations to further reduce code size with the risk of sacrificing performance
- -Og - enables most optimizations from -O1, but may disable some optimizations to improve debuggability.



Offload floating point calculations to R5 FPU

- Use -mfpu=vfpv3-d16


Decide Thumb vs ARM mode based on profiling
 - Compiler option -mthumb
  - Instruct the compiler to generate THUMB mode instructions
 - Compiler option -marm
  - Instruct the compiler to generate ARM mode instructions


### Optimize application code {#CHAPTER_OPTIMIZATION_SECTION_5}

1. Unroll loops – for loops and nested if’s
2. Avoid repeated peripheral reads by maintaining a local copy (in TCM) of register values. And use in ISR.
3. Use MCU SDK R5 trig math library functions for Trigonometric calculations
4. Contiguous allocation of ADC results to take advantage of 32-it read to get two 16-bit ADC results
5. Program specific parts of the control loop using R5F assembly to get greater control over the access pattern and sequence to achieve better utilization of R5F cycles.
6. IPC read buffers to be allocated from memory closer to CPU. R5F TCM, ICSS DMEM, M4 TCM etc


SoC hardware features:
1. Enhanced topology for non-conflicting access to the significant number of control peripherals from multiple R5F cores (discussed below)
2. Enhanced interconnect for
 - Low latency read access to analog sensing peripherals (ADC results)
 - Low latency write access to control/actuation peripherals (EPWM comparators)
 - Supporting higher throughput (back to back writes) to control peripherals


#### Peripheral Bus Architecture & Mapping

CONTROLSS interconnect is divided into below list of separate interconnect connected to the CORE VBUSP interconnect individually. 
Since this are connected to the CORE VBUSP interconnect separately, each of this interconnect can be accessed in parallel by different initiators without any arbitration. 
Accessing a single CONTROLSS interconnect by multiple initiators at the same time will be arbitrated.

- MISC PERIPH
- MISC CONFIG
- FSI0 (FSITX[0:1] and FSIRX[0:1])
- FSI1 (FSITX[2:3] and FSIRX[2:3])
- G0_EPWM, G1_EPWM, G2_EPWM, G3_EPWM
- ADC0, ADC1, ADC2, ADC3, ADC4, ADC5

Below diagram shows the different interconnect connections.

\imageStyle{Fig10.PNG,width:80%}
 \image html Fig10.PNG "Figure 10"


MISC PERIPH, MISC CONFIG, FSI0 and FSI1 are single initiator, multiple targets as shown in the diagram.
EPWM interconnect are divided into 4 groups G0_EPWM, G1_EPWM, G2_EPWM and G3_EPWM accessed using different address regions in the memory map. 
Each interconnect has n target ports depending on number of EPWM in the design. 
After the interconnect, a 4:1 Static Mux can be configured per EPWM using CONTROLSS_CTRL.EPWM_STATICXBAR_SEL0 & CONTROLSS_CTRL.EPWM_STATICXBAR_SEL1 register.

ADC0, ADC1, ADC2, ADC3, ADC4 and ADC5 are different interconnect per intiator (R5FSS0-0_AHB, R5FSS0-1_AHB,R5FSS1-0_AHB, R5FSS1-1_AHB, CORE VBUSP (Port0), and CORE VBUSP (Port1)). 
The target ports are based on number of ADCs in the design. 
Each initiator can independently access any ADC register without any arbitration.


### Memory optimization {#CHAPTER_OPTIMIZATION_SECTION_6}

Code Size optimizations
- The -Oz option is recommended for optimizing code size.
- The -O3 option is recommended for optimizing performance, but it is likely to increase compiler generated code size.
- ARM/ Thumb
 - -mthumb
  - Instruct the compiler to generate THUMB mode instructions (16-bit THUMB or T32 THUMB depending on which processor variant is selected) for current compilation
 - -marm
  - Instruct the compiler to generate ARM mode instructions for current compilation
