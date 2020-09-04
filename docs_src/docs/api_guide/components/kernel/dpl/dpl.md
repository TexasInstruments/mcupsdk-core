# Driver Porting Layer (DPL) {#KERNEL_DPL_PAGE}

[TOC]

# Introduction

This module contains APIs which are used by the drivers to make them agnostic of the underlying OS and CPU architecture.

It consists of below sub-modules

- \subpage KERNEL_DPL_HWI_PAGE
- \subpage KERNEL_DPL_SEMAPHORE_PAGE
- \subpage KERNEL_DPL_TASK_PAGE
- \subpage KERNEL_DPL_CLOCK_PAGE
- \subpage KERNEL_DPL_CYCLE_COUNTER_PAGE
- \subpage KERNEL_DPL_QUEUE_PAGE

\cond SOC_AM64X || SOC_AM243X || SOC_AM273X || SOC_AWR294X || SOC_AM263X
- \subpage KERNEL_DPL_CACHE_PAGE
- \subpage KERNEL_DPL_CPUID_PAGE
\endcond
\cond SOC_AM64X
- \subpage KERNEL_DPL_MMU_ARMV8_PAGE
\endcond
\cond SOC_AM64X || SOC_AM243X || SOC_AM273X || SOC_AWR294X || SOC_AM263X || SOC_AM62X
- \subpage KERNEL_DPL_MPU_ARMV7_PAGE
\endcond
- \subpage KERNEL_DPL_ADDR_TRANSLATE_PAGE
- \subpage KERNEL_DPL_HEAP_PAGE
- \subpage KERNEL_DPL_DEBUG_PAGE
- \subpage KERNEL_DPL_EVENT_PAGE
- \subpage KERNEL_DPL_TIMER_PAGE
