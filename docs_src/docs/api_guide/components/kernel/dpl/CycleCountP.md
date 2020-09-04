# Cycle Counter {#KERNEL_DPL_CYCLE_COUNTER_PAGE}

[TOC]

## Features Supported

- API to return CPU cycles executed so far
- API to reset the CPU cycle counter

## Features NOT Supported

- Additional event counters in the CPU are not supported via this API. 
  Refer to CPU architecture specific technical and architecture reference manuals, to access additional CPU architecture specific counter features.

## Important Usage Guidelines

- Make sure to call CycleCounterP_reset() to enable and reset the CPU cycle counter before using it
- CycleCounterP_getCount32() uses a CPU architecture specific counter to count every CPU cycle.
  Since this is a 32b counter, this will typically wraparound and overflow within few seconds.
  E.g. for 500Mhz CPU, the counter will overflow in ~ 8secs.
- Application can have logic to handle one wraparound of the counter, as shown in the example usage below.
  i.e max time duration that can be measured will be limited to 0xFFFFFFFF CPU  cycles, i.e few seconds only.
- This API is meant to used for very fine, short duration measurements. To measure longer durations
  use \ref ClockP_getTimeUsec() from \ref KERNEL_DPL_CLOCK_PAGE module.

## Example Usage

Include the below file to access the APIs,
\snippet CycleCounterP_sample.c include

Example usage for counting CPU cycles,
\snippet CycleCounterP_sample.c usage

## API

\ref KERNEL_DPL_CYCLE_COUNTER
