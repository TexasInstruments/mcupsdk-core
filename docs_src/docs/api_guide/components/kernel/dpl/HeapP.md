# Heap {#KERNEL_DPL_HEAP_PAGE}

[TOC]

## Features Supported

- Ability to create multiple user defined heaps
- Alloc and free APIs 
- Detailed heap statistics
- Thread safe APIs with RTOS
- Minimum alignment of \ref HeapP_BYTE_ALIGNMENT for all allocations

## Features NOT Supported

- Memory allocation with alignment as a input. Users should allocate extra and align after allocation.

## Important Usage Guidelines

NA

## Example Usage

Include the below file to access the APIs,
\snippet HeapP_sample.c include

Example usage to define a heap memory and heap handle
\snippet HeapP_sample.c heap def

Example usage to create a heap,
\snippet HeapP_sample.c create

Example usage to alloc and free memory,
\snippet HeapP_sample.c alloc

## API

\ref KERNEL_DPL_HEAP
