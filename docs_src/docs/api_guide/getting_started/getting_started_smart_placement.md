# Getting Started With Smart Placement {#SMART_PLACEMENT_GETTING_STARTED}
[TOC]
## Introduction

Usually used when application foot print is more than internal memory size. More on this can be read at \ref SMART_PLACEMENT

### When is smart placement required:

Smart Placement would:
1. Place critical code section needed for deterministic latencies to appropriate memories
2. Identify and place code section which are bottleneck to system performance and place them in faster memory.
3. Improves XIP performance.

### What not to expect from Smart Placement?

Smart placement is dependent on profiling data and the input given by the user on the priority of the function. The result achieved by smart placement tool is dependent on these factors. In absence of this factor, the full effect of smart placement would not be achieved.

## Specific implementation of smart placement

### Manual Implementation

Use this when
1. it is not suitable to run the application on the target and collect runtime profiling data.
2. Developer of the application have knowledge about the criticality of various functions/Tasks.

More on this can be read at \ref MANUAL_SMART_PLACEMENT

### SW Instrumentation implementation

This method gives an automated way of finding critical functions. A more frequent function is more critical function. Use this when:
1. Application runtime profiling and profiling data extraction is possible.
2. Higher profiling coverage is required.

More on this can be read at \ref SW_INSTRUMENT_SMART_PLACEMENT
