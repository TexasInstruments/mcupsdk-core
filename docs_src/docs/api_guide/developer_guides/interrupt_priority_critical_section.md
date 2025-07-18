# Interrupt Priority Mask based Critical Sections for R5F {#INTERRUPT_PRIORITY_BASED_CRITICAL_SECTIONS_R5F}

[TOC]

## Introduction

Entering a critical section disables all interrupts globally by default, ensuring atomicity but increasing interrupt
latency. Interrupt Priority Mask based Critical Sections provide a more flexible approach: only interrupts below a
certain priority are masked (disabled), while higher-priority interrupts remain enabled even within a critical section.
This allows critical real-time events to be serviced promptly, improving system responsiveness.

\attention
The interrupt priority based critical sections specified below is specific to ARM R5F architecture.

\note  
Interrupt priority mask based critical section have some overheads in taking critical section. <br>
So unless this feature is really needed keep this disabled.

## FreeRTOS Use Case

### Enabling Interrupt Priority Mask Based Critical Sections

To enable this feature in FreeRTOS, update your `FreeRTOSConfig.h`:

```c
#define configUSE_INTERRUPT_PRIORITY_BASED_CRITICAL_SECTIONS    (1)
#define configMAX_SYSCALL_INTERRUPT_PRIORITY                    (0x4U) // Example value
```

- `configUSE_INTERRUPT_PRIORITY_BASED_CRITICAL_SECTIONS`:
  - Set to `1` to enable priority-based masking
- `configMAX_SYSCALL_INTERRUPT_PRIORITY`:
  - Sets the priority threshold
    - Higher priority interrupts (numerically lower priority value) remain enabled in the critical section
    - Low priority interrupts (numerically greater than or equal priority value) are masked (disabled)

\note
After updating the config, rebuild all FreeRTOS libraries and your application.

### Entering and Exiting Critical Sections (FreeRTOS)

- Use `taskENTER_CRITICAL()` and `taskEXIT_CRITICAL()` in task context.
- Use `taskENTER_CRITICAL_FROM_ISR()` and `taskEXIT_CRITICAL_FROM_ISR()` in ISR context.

#### Nested Critical Sections (FreeRTOS)

- **Tasks:**
  - FreeRTOS maintains a nesting count in the task's TCB.
  - Nested calls are supported and only the outermost `EXIT` re-enables masked interrupts.
- **ISRs:** 
  - The `taskENTER_CRITICAL_FROM_ISR()` returns a key representing the previous mask state.
  - Use this key with `taskEXIT_CRITICAL_FROM_ISR()` to restore the state.
  - There is no nesting count; instead, a set/restore approach is used.

\warning
Critical ISRs (Higher-priority - those with a numerically lower priority value than `configMAX_SYSCALL_INTERRUPT_PRIORITY`)
must not enter a critical section. Doing so can cause undefined behavior. <br>
If you really need to mask all interrupts, use `HwiP_disable()`. <br>
Also these ISRs shouldn't use any FreeRTOS (`FromISR`) APIs, since this would attempt to enter a critical section.
FreeRTOS checks this with `portASSERT_IF_INTERRUPT_PRIORITY_INVALID()`.

#### Pseudocode Example (Task Context)

```c
void task_func(void) 
{
    taskENTER_CRITICAL();
    // Critical section: lower-priority interrupts are masked (priority value >= configMAX_SYSCALL_INTERRUPT_PRIORITY)
    //                   higher-priority interrupts are still enabled (priority value < configMAX_SYSCALL_INTERRUPT_PRIORITY)
    // ... code ...
    taskEXIT_CRITICAL();
}
```

#### Pseudocode Example (ISR Context, with nested interrupts enabled)

```c
// ISR function of an interrupt with priority value say 'n' 
// such that `configMAX_SYSCALL_INTERRUPT_PRIORITY <= n < HwiP_MAX_PRIORITY`
void isr_func(void)
{
    // Since in an ISR, all interrupts of priority less than or equal to current priority are already masked.
    // i.e., lower-priority interrupts are masked (priority value >= n)
    //       higher-priority interrupts are still enabled (priority value < n)
    UBaseType_t key = taskENTER_CRITICAL_FROM_ISR();
    // Critical section in ISR: lower-priority interrupts are masked (priority value >= configMAX_SYSCALL_INTERRUPT_PRIORITY)
    //                          higher-priority interrupts are still enabled (priority value < configMAX_SYSCALL_INTERRUPT_PRIORITY)
    // ... code ...
    taskEXIT_CRITICAL_FROM_ISR(key);
}
```

## NoRTOS Use Case

For NoRTOS, enable this feature by defining `HWIP_USE_INTERRUPT_PRIORITY_BASED_CRITICAL_SECTIONS` in `HwiP.h`:

```c
#define HWIP_USE_INTERRUPT_PRIORITY_BASED_CRITICAL_SECTIONS
#define HWIP_CRITICAL_SECTION_INTERRUPT_PRIORITY_THRESHOLD    (4U) // Example value
```

- `HWIP_CRITICAL_SECTION_INTERRUPT_PRIORITY_THRESHOLD`:
  - Sets the priority threshold for masking.
  - This is similar to `configMAX_SYSCALL_INTERRUPT_PRIORITY` in FreeRTOS.

### Entering and Exiting Critical Sections (NoRTOS)

- Use `HWIP_ENTER_CRITICAL_SECTION()` to enter and `HWIP_EXIT_CRITICAL_SECTION(key)` to exit.
- These APIs use a set/restore approach, allowing nested critical sections.
  - The key returned by `ENTER` must be passed to `EXIT` to restore the previous state.

\note
These APIs can be used from both main context and ISRs.

\warning
However, do not use them from a Critical ISR (Higher-priority - those with a numerically lower priority value than
`HWIP_CRITICAL_SECTION_INTERRUPT_PRIORITY_THRESHOLD`), as this may cause unexpected behavior. <br>
If you really need to mask all interrupts, use `HwiP_disable()`.

#### Pseudocode Example

```c
void main_or_isr(void) {
    uintptr_t key = HWIP_ENTER_CRITICAL_SECTION();
    // Critical section: lower-priority interrupts are masked (priority value >= HWIP_CRITICAL_SECTION_INTERRUPT_PRIORITY_THRESHOLD)
    //                   higher-priority interrupts are still enabled (priority value < HWIP_CRITICAL_SECTION_INTERRUPT_PRIORITY_THRESHOLD)
    // ... code ...
    HWIP_EXIT_CRITICAL_SECTION(key);
}
```

## Example Case Study: Deep Nested Critical Sections

This section presents a detailed, step-by-step walkthrough of an example scenario that demonstrates the behavior of
interrupt priority mask based critical sections. It illustrates how critical sections — entered from both task and ISR
contexts — interact with interrupts at different priority levels. By following this example scenario, you will gain a
deeper understanding of how interrupt priority masking operates in practice, particularly in complex situations 
involving deeply nested interrupts.

### Scenario Overview

- The system is configured with multiple interrupt priority levels (16 levels), and the critical section priority
  mask is set to 4.
- Critical ISRs (priority < mask threshold) are allowed to run even inside the critical section,
  while Normal ISRs (priority ≥ mask threshold) are masked and deferred until the critical section is exited.

#### Additional Notes on Interrupt Triggering and Execution Order

- A task enters a critical section and triggers two interrupts:
  - The lowest priority normal interrupt (priority 15)
  - The lowest priority critical interrupt (priority 3, which is just above the mask threshold)
- The system will immediately switch to the critical ISR when it is triggered

- Each critical ISR will trigger the next higher priority critical interrupt as well as next higher priority normal
  interrupt
- The system will immediately switch to the higher priority critical ISR when it is triggered
- All normal ISRs will be executed only after exiting from the task critical section,
  in order of their pending interrupt priorities

- Each normal ISR, when executed, will trigger the next higher priority normal interrupt (if it has not already
  executed) from within its ISR critical section
- After exiting the ISR critical section, the system will immediately switch to the higher priority normal ISR, 
  preempting the current ISR

These behaviors ensure that critical ISRs are always serviced promptly, while normal ISRs are deferred and executed in
priority order once the critical section is exited.

### Step-by-Step Execution

1. **Task Context:**
    - The task enters a critical section using `taskENTER_CRITICAL()`
    - It posts (triggers) the lowest priority normal interrupt (priority 15). This interrupt is masked and will not
      execute until the critical section is exited.
    - It then posts the lowest priority critical interrupt (priority 3). Since this interrupt is above the mask
      threshold, it is allowed to execute immediately, preempting the task.

2. **Critical ISR Execution:**
    - The critical ISR for priority 3 runs
      - Inside this ISR, it posts the next higher priority normal interrupt (priority 14) and the next higher priority
        critical interrupt (priority 2)
      - The normal interrupt (priority 14) is masked and will not execute yet
      - The critical interrupt (priority 2) is above the mask threshold and executes immediately, preempting the current
        ISR of priority 3
    - The critical ISR for priority 2 runs
      - It posts the next higher priority normal interrupt (priority 13) and the next higher priority critical interrupt
        (priority 1)
      - The normal interrupt (priority 13) is masked
      - The critical interrupt (priority 1) executes immediately, preempting the current ISR of priority 2
    - The critical ISR for priority 1 runs
      - It posts the next higher priority normal interrupt (priority 12) and the next higher priority critical interrupt
        (priority 0)
      - The normal interrupt (priority 12) is masked
      - The critical interrupt (priority 0) executes immediately, preempting the current ISR of priority 1
    - The critical ISR for priority 0 runs
      - It posts the next higher priority normal interrupt (priority 11)
      - The normal interrupt (priority 11) is masked
      - No further critical interrupts are posted, so the ISR returns
    - After the highest priority critical ISR (priority 0) completes, the system unwinds and returns from each nested
      critical ISR in reverse order (priority 1, 2, 3), as there are no more higher-priority critical interrupts pending

3. **Exiting Task Critical Section:**
    - After all critical ISRs have returned, the task exits the critical section using `taskEXIT_CRITICAL()`
    - Now, all pending normal interrupts (priorities 11–15) are unmasked and can execute

4. **Normal ISR Execution:**
    - The highest priority pending normal ISR (priority 11) executes first
    - Each normal ISR enters a critical section using `taskENTER_CRITICAL_FROM_ISR()`, triggers the next higher priority
      normal interrupt (i.e, priority 10 interrupt from ISR for 11, priority 9 interrupt from ISR for 10, etc.), and 
      then exits the critical section using `taskEXIT_CRITICAL_FROM_ISR()`
    - Upon exiting the critical section, the system immediately switches to the next higher priority normal ISR,
      preempting the current ISR
    - This process repeats until the highest priority normal ISR (priority 4) is entered.
    - After entering the ISR for priority 4, the system begins to unwind, returning from each ISR in reverse order: 4,
      5, 6, 7, 8, 9, 10, and finally 11
    - After exiting from 11, the system then enters and exits the next pending normal ISRs
      (priorities 12, 13, 14, and 15) one by one, each time entering and immediately exiting the ISR

5. **Completion:**
    - After all ISRs have executed, control returns to the task.

### Execution Flow

\image html deep_nested_critical_sections.svg "Execution Flow for Deep Nested Critical Section Example Scenario"

### Pseudocode

```c
static void critical_isr(void *arg)
{
    // compute `nextNormalIntNum` & `nextCriticalIntNum` based on `arg`

    /* Trigger the next higher priority normal interrupt. */
    HwiP_post(nextNormalIntNum);

    /* Trigger the next higher priority critical interrupt. This should fire immediately. */
    HwiP_post(nextCriticalIntNum);
}

static void normal_isr(void *arg)
{
    // compute `nextNormalIntNum` based on `arg` and check if already executed or not

    key = taskENTER_CRITICAL_FROM_ISR(); /* Enter Critical Section */

    /* Trigger the next higher priority normal interrupt(if not already executed).
     * This should fire soon after the critical section exit from ISR below. */
    HwiP_post(nextNormalIntNum);
  
    taskEXIT_CRITICAL_FROM_ISR(key); /* Exit Critical Section */
}

void test_task(void *args)
{
    // Register interrupts

    taskENTER_CRITICAL();

    /* Trigger lowest priority normal interrupt. 
     * This shouldn't fire until exit from critical section as well as all other high priority pending interrupts are serviced. */
    HwiP_post(NORMAL_ISR_LOWEST_PRI_INT_NUM);
    /* Trigger the lowest priority critical interrupt (priority still higher than normal interrupts)
     * This should fire immediately */
    HwiP_post(CRITICAL_ISR_LOWEST_PRI_INT_NUM);

    /* Each critical ISR triggers the next higher critical interrupt as well as normal interrupts.
     * It should switch to higher priority critical ISR immediately.
     * All normal ISR should be executed only after exit from the task critical section. */

    taskEXIT_CRITICAL();

    /* All normal ISRs should get executed now */
    /* Each normal ISR triggers the next higher normal interrupt (if not already executed). */

    // Unregister interrupts
}
```

### Key Observations

- **Critical ISRs** (priority above the mask threshold) can preempt both the task and other ISRs, and are executed
  immediately, even within a critical section.

- **Normal ISRs** (priority below or equal to the mask threshold) are deferred until the critical section is exited,
  regardless of how many times they are posted.

- The execution order strictly follows interrupt priorities, with higher-priority ISRs always preempting lower-priority
  ISR code.

- This scenario demonstrates how nested critical sections and interrupt priorities interact, ensuring that only the most
  urgent events are serviced immediately, while less critical events are deferred for atomicity.
