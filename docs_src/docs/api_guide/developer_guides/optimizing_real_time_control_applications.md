# Optimizing Real Time Control Applications {#OPTIMIZING_REAL_TIME_CONTROL_APPLICATIONS}

[TOC]

## About this document {#CHAPTER_ABOUT}

This guide describes the performance optimizations of real-time applications on the Texas Instruments @VAR_SOC_NAME platform.

TI’s Sitara MCU processor @VAR_SOC_NAME is designed for real-time control applications, specifically for use cases such as Traction inverters, Onboard chargers, Charging stations, DC-DC converters, Industrial AC inverters, String inverters and Industrial communication.
Crucial aspect of such applications is the signal chain performance. Latency of the signal chain determines the efficiency and performance of the system.

The goal of this document is to provide a comprehensive guide to performance optimization on the @VAR_SOC_NAME platform and help real-time application developers achieve their performance goals.

1. Signal chain performance considerations. See \ref CHAPTER_OPERATION
2. List of \ref CHAPTER_OPTIMIZATION
 - Software, SoC hardware and application
 - Best practices, recommendations and step-by-step guide


## Real Time Control Loop {#CHAPTER_OPERATION}


- Crucial aspect of Real Time Control applications is signal chain performance – which includes sensing, processing and actuation. Latency of the signal chain determines the efficiency and performance of the system (for example: RPM of traction motor in automotive EV application).
 - Sensing uses on-chip analog to digital converters, comparators,..
 - Actuation uses PWM generation modules
 - Processing involves the CPU core (R5F in @VAR_SOC_NAME) running control loop algorithm


\imageStyle{Fig1.PNG,width:80%}
 \image html Fig1.PNG "Signal chain"



### Control Interrupts and ISR {#CHAPTER_OPERATION_SECTION_0}

Interrupt Service Routines, commonly referred to as ISRs, play a critical role in real-time control applications. 
Control ISRs are often used in real-time control systems to read sensed (typically ADC) inputs, perform calculations/run algorithms (called Control loop/Control algorithm in this document) and write to outputs (typically PWM)

\imageStyle{Fig2.PNG,width:80%}
 \image html Fig2.PNG "Various operations in Real Time Control interrupt"


As shown in figure above, 5 operations are

- \ref CHAPTER_OPERATION_SECTION_1
- \ref CHAPTER_OPERATION_SECTION_2
- \ref CHAPTER_OPERATION_SECTION_3
- \ref CHAPTER_OPERATION_SECTION_4
- \ref CHAPTER_OPERATION_SECTION_5



\imageStyle{Fig3.PNG,width:80%}
 \image html Fig3.PNG "Operation 2 to 5 in a typical application"


Following sections show optimizations relevant for each operation performed in the Control loop ISR
- Description about the operation and performance consideration
- Software and hardware optimization options


### Operation 1: Latch and respond to interrupt {#CHAPTER_OPERATION_SECTION_1}

\note Latency to latch and repond to interrupt from ControlSS is 29 cycles (worst case) at 400MHz R5F core clock.

This latency depends on
1. Propagation of interrupt from peripheral to R5F. This is predominantly dependent on hardware latencies.
2. Instruction under execution in R5F. Worst case scenario is when R5F starts executing a multi-cycle instruction and unable to respond to the interrupt. Example: a memory load instruction (from OCRAM) under execution in background loop or a low priority task when expecting an ADC interrupt

Optimization options: 
1. ADC interrupt
 - Use Early interrupt feature of ADC to triggers R5F interrupt ahead of completion of ADC conversion. Part of interrupt latency can be absorbed in ADC conversion latency
2. EPWM interrupt
 - Use Counter Compare to offset the EPWM interrupt

\imageStyle{Fig4.PNG,width:80%}
 \image html Fig4.PNG "Interrupt route"

### Operation 2: R5F Context save {#CHAPTER_OPERATION_SECTION_2}

\note Latency to save context is 53 cycles (best case) at 400MHz R5F core clock.

This latency depends on 
1. Saving/preserving the context of interrupted task
 - R5F Program status register, LR, core registers
 - This latency is predominantly dependent on software/compiler generated code.
2. Support for floating point in the interrupt routine
 - VFP control status: FPEXC, FPSCR, VFP double-precision float: D0-D7
3. Support for nesting of interrupts


Refer \ref CHAPTER_OPTIMIZATION for list of optimization options and trade-offs for this operation.


\imageStyle{Fig5.PNG,width:80%}
 \image html Fig5.PNG "Code for context save"

\imageStyle{Fig6.PNG,width:80%}
 \image html Fig6.PNG "Context save"


### Operation 3: R5F read accesses to sensed inputs {#CHAPTER_OPERATION_SECTION_3}

\note Latency to read inputs (one 16-bit ADC result) is 18 cycles (best case) at 400MHz R5F core clock

This latency depends on 
1. Hardware latency. ControlSS peripheral register read access
2. Software/code. Pattern of accessing the registers
3. Data type conversion (for example: int to float)


For example: 
In motor control use cases, R5 reads 16bit ADC results, converts results to floating point values and writes the values to memory for further calculation

Optimization options and trade-offs:
- 2 byte or 4 byte ADC read from R5F (a 32 bit core) involve same latency (18 cycles). 
 - So, as an optimization use consecutive ADC SOCs, so that  that 2 16bit ADC results are stored consecutively in a 32bit register. This reduces 2 read accesses.
 - Refer below the code used to split the 32 bit result to 2 16 bit values
- For conversion to floating point, use R5 FPU.
- Write to TCM (1 cycle) instead of L2 OCRAM
- Alternate approach is to use DMA to bring the ADC results to TCM. This can absorb overheads for R5 to read ADC results.

\imageStyle{Fig7.PNG,width:80%}
 \image html Fig7.PNG "ADC read access optimization"

### Operation 4: R5F executes calcuations/algorithm {#CHAPTER_OPERATION_SECTION_4}

\note Execution time of the control algorithm is specific to application use case. 
	For example: Field Oriented Control computations for @VAR_SOC_NAME traction inverter executes for 985ns. Refer application note. SPRAD32
    https://www.ti.com/lit/an/sprad32/sprad32.pdf

This depends on 
1. Number of inputs and outputs required. For example: number of channels, phases, stages
2. Number of floating point/trigonometric calculations


Optimization options and trade-offs:

- \ref CHAPTER_OPTIMIZATION_SECTION_2
- \ref CHAPTER_OPTIMIZATION_SECTION_4
- \ref CHAPTER_OPTIMIZATION_SECTION_5

\imageStyle{Fig8.PNG,width:80%}
 \image html Fig8.PNG "Execution time for FOC computation from SPRAD32"

### Operation 5: R5F writes to outputs {#CHAPTER_OPERATION_SECTION_5}

\note Throughput to write outputs (For example: PWM compare values) is 2 cycles per write (best case) at 400MHz R5F core clock. Refer section CHAPTER_OPTIMIZATION_SECTION_3 on the settings to achieve this throughput.

This latency depends on 
1. Hardware latency (write access latency) 
2. Software/Generated code (pattern of read/write)

 - Load from TCM consumes 1 R5 cycle. Store consumes 2 R5 cycles (1 bus cycle @200MHz).
 - So 3 cycles for read+write.

Optimization options and trade-offs:

- \ref CHAPTER_OPTIMIZATION_SECTION_4
- Setup the PWM region as device memory (not strongly ordered). Refer section CHAPTER_OPTIMIZATION_SECTION_3. This helps to achiee 2 cycle write throughput.
- Store all the PWM data (compare values for modulation) in TCM consecutively for quick access
- Interleave reads and writes (interleave load and store instructions). TI ARM CLANG (used by SDK) supports this optimization
 - When the PWM writes are posted writes, the 2nd unused cycle of store instruction can be used for next read operation. This reduces overall write throughput to 2 cycles


\imageStyle{Fig9.PNG,width:80%}
 \image html Fig9.PNG "PWM write latency optimization"





