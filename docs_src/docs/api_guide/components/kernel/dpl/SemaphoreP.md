# Semaphore {#KERNEL_DPL_SEMAPHORE_PAGE}

[TOC]

## Features Supported
- APIs to create binary and counting semaphore's
- APIs to create mutual exclusion semaphore
- Nesting of mutual exclusion semaphores
- Priority inheritance when supported by the underlying OS

## Features NOT Supported

NA

## Important Usage Guidelines

- \ref SemaphoreP_post can be called from ISR as well as task context
- \ref SemaphoreP_pend should not be called within ISR context. It can be called from task context.
- \ref SemaphoreP_pend will block until the user specific timeout ticks have expired or semaphore is acquired.
  - In baremetal case, there is only a single main task and that will block or spin until the ticks have elasped or semaphore acquire is successful
  - In RTOS case, the current executing task will 'pend' and schedular will switch to another ready task until the ticks have elasped or semaphore acquire is successful
  - In both cases ISR's are still active
- There is nothing like priority inheritance in no-RTOS mode

## Example Usage

Include the below file to access the APIs,
\snippet SemaphoreP_sample.c include

Example usage to define a semaphore
\snippet SemaphoreP_sample.c define

Example usage to post a binary semaphore from a ISR
\snippet SemaphoreP_sample.c callback

Example usage to create and use a mutex
\snippet SemaphoreP_sample.c mutex

Example usage to create and use a binary semaphore with timeout
\snippet SemaphoreP_sample.c binary

Example usage to create and use a counting semaphore
\snippet SemaphoreP_sample.c counting

## API

\ref KERNEL_DPL_SEMAPHORE