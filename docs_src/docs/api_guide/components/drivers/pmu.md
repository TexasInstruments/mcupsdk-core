# PMU {#DRIVERS_PMU_PAGE}

[TOC]

The ARM Cortex R5F core includes logic to detect various events that can occur, 
for example a cache miss, cache access, etc. These events provide useful 
information about the behaviour of the processor that you can use when debugging 
or profiling code. The events are made visible on an output event bus and can be 
counted using registers in the Performance Monitoring Unit (PMU) of the core. 
This SW library is an abstraction to the PMU with APIs to profile code blocks.

The PMU consists of three event counting registers, one cycle counting register 
and 12 CP15 registers, for controlling and interrogating the counters. So in total 
four profiling metrics are possible for each profile point. Out of this one is 
fixed, the cycle counter. The other three are programmable to any of the events 
supported by the R5 core. For details of the events, please refer to the ARM Cortex 
R5F technical reference manual under section "Events and Performance Monitor"

## Features Supported

The driver consists of a high-level interface offering the following features:

- API to initialize profile object and counters
- APIs to start and end profiling
- APIs to print stats of a specific profile point or the all the profile points

## Features NOT Supported

- Sysconfig configurability of counters is not supported
- Counter overflow is not handled (counters are reset for each profile point)

## Example Usage

Include the below file to access the APIs
\snippet Pmu_sample.c include

Sample configuration
\snippet Pmu_sample.c config

Initialization
\snippet Pmu_sample.c init

Profiling sample
\snippet Pmu_sample.c profile

For more details on usage, please refer \ref EXAMPLES_DRIVERS_PMU_MULTIEVENT

## API

\ref DRV_PMU_MODULE