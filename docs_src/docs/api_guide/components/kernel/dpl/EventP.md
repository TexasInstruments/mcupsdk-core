# Event {#KERNEL_DPL_EVENT_PAGE}

[TOC]

## Features Supported
- APIs to construct and destruct Events
- APIs to set, get or clear bits and wait for bits

## Features NOT Supported

- These APIs are not implemented for no-RTOS. Error will be returned if these APIs called using no-RTOS.

## Important Usage Guidelines

- \ref EventP_waitBits should not be called within ISR context. It can be called from task context.
- Setting or clearing the Event bits is a non-deterministic operation and FreeRTOS does not allow non-deterministic operations to be performed in interrupts or from critical sections. If \ref EventP_setBits or \ref EventP_clearBits is called from an ISR, a message is sent to RTOS daemon task to perform the operation in the context of daemon task. If the set or clear operation is required to be completed immediately, then the priority of RTOS daemon task must be higher than the priority of task waiting for the Event bits. The priority of RTOS daemon task is set by the `configTIMER_TASK_PRIORITY` in FreeRTOSConfig.h file.
-  The actual current value of Event bits returned from \ref EventP_waitBits and \ref EventP_getBits can be different from returned value if an interrupt or a higher priority task modifies the value between the calling task leaving blocked state and exiting this API.

## Example Usage

Include the below file to access the APIs
\snippet EventP_sample.c include

Example usage to define an Event
\snippet EventP_sample.c define

Example usage to create an Event
\snippet EventP_sample.c create

Example usage to set and wait for bits across two tasks
\snippet EventP_sample.c task1
\snippet EventP_sample.c task2

## API

\ref KERNEL_DPL_EVENT