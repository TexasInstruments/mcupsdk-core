# SPINLOCK {#DRIVERS_SPINLOCK_PAGE}

[TOC]

The Spinlock driver provides API to program the Spinlock module
to synchronize processes running on multiple processors in the device.

## Features Supported

- 256 spinlocks
- Lock and unlock API

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

- Not applicable/supported

## Features NOT Supported

NA

## Important Usage Guidelines

- The Spinlock driver doesn't enforce below limitation as defined in device manual.
It is the responsibility of the application to take care of below recommendations
- Spinlock is appropriate for mutual exclusion for access to a shared data structure. It should be used only when:

    - The time to hold the lock is predictable and small (for example, a maximum hold time of less than 200 CPU
      cycles may be acceptable).
    - The locking task cannot be preempted, suspended, or interrupted while holding the lock (this would make the
      hold time large and unpredictable).
    - The lock is lightly contended, that is the chance of any other process (or processor) trying to acquire the lock
      while it is held is small.

## Example Usage

Include the below file to access the APIs
\snippet Spinlock_sample.c include

Lock API
\snippet Spinlock_sample.c lock

Unlock API
\snippet Spinlock_sample.c unlock

## API

\ref DRV_SPINLOCK_MODULE
