# Task {#KERNEL_DPL_TASK_PAGE}

[TOC]

\attention This module is not applicable for NO RTOS environment

## Features Supported
- APIs to create and destroy tasks
- APIs to yeild a task
- Ability to specify task priority, task entry function, stack pointer, stack size
- Ability to get task and CPU load

## Features NOT Supported

NA

## Important Usage Guidelines

- On R5F, and M4F, make sure memory provided as stack is 32b aligned and size is also multiple of 32b
- Stack and stack size MUST be provided by application and is not allocated internally

## Example Usage

Include the below file to access the APIs,
\snippet TaskP_sample.c include

Example usage to define task objects and parameters
\snippet TaskP_sample.c define

Example task main function
\snippet TaskP_sample.c taskmain

Example usage to create a task
\snippet TaskP_sample.c create

Example usage to get task and CPU load
\snippet TaskP_sample.c load

## API

\ref KERNEL_DPL_TASK