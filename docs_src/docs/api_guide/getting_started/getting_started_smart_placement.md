# Smart Placmenet {#SMART_PLACEMENT_GETTING_STARTED}
[TOC]
## Introduction

Smart placement is to enable profiling based application optimization.

Usually used when application foot print is more than  internal memory size (2.5MB for case of AM263Px) smart placement can be used. More on this can be read at \ref SMART_PLACEMENT

The basic assumption is that application needs to be profiled to gather runtime information. This information is then used to optimize the applcaition by moving functions from external slower, but larger, flash to internal memory which is faster but small in size.

Smart placemnet provide tools and method to make the process easy via specific implementaions of smart placement

## Specific implementation of smart placement

### Manual Implementation

Use this when
1. it is not suitable to run the application on the target and collect runtime profiling data.
2. Developer of the applicaiton knows the applicaiton.

More on this can be read at \ref MANUAL_SMART_PLACEMENT

## SW Instrumentation implementaiton

This method gives an automated way of finding critical functions. A more frequent function is more critical function. Use this when:
1. Application runtime profiling and profiling data extraction is possible.
2. Higher profiling coverage is required.

More on this can be read at \ref SW_INSTRUMENT_SMART_PLACEMENT