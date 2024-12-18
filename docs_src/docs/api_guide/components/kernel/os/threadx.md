# ThreadX {#KERNEL_THREADX}

[TOC]

## Introduction

ThreadX is a high-performance real-time kernel designed specifically for embedded applications.

Unlike other real-time kernels, ThreadX is designed to be versatile—easily scaling among small microcontroller-based applications through those that use powerful CISC, RISC, and DSP processors.

ThreadX is scalable based on its underlying architecture. Because ThreadX services are implemented as a C library, only those services actually used by the application are brought into the run-time image. Hence, the actual size of ThreadX is completely determined by the application. For most applications, the instruction image of ThreadX ranges between 2 KBytes and 15 KBytes in size.

- \ref EXAMPLES_KERNEL_THREADX_HELLO_WORLD
- \ref EXAMPLES_KERNEL_THREADX_TASK_SWITCH

For the full ThreadX API reference, please refer to the below table.

## Additional References {#KERNEL_THREADX_ADDITIONAL_REFERENCES}

<table>
<tr>
    <th>Website Link
    <th>Description
</tr>
<tr>
    <td>[ThreadX kernel](https://github.com/eclipse-threadx/rtos-docs/blob/main/rtos-docs/threadx/about-this-guide.md)
    <td>Complete documentation of the ThreadX kernel and API references
</tr>
</table>
