# Cache {#KERNEL_DPL_CACHE_PAGE}

[TOC]

## Features Supported

- APIs to perform below cache operations
  - region based writeback
  - region based invalidate
  - region based writeback invalidate
  - enable and disable APIs for I cache and D cache
  - full cache writeback, write invalidte
\cond SOC_AM64X
- CPUs supported,
  - R5F, A53
  - M4F, blank implementation since no cache on M4F
\endcond
\cond SOC_AM243X
- CPUs supported,
  - R5F
  - M4F, blank implementation since no cache on M4F
\endcond
## Features NOT Supported

NA



## Important Usage Guidelines

- The region based cache APIs, \ref CacheP_wb, \ref CacheP_wbInv, \ref CacheP_inv, operate on a CPU cache line, hence its strongly recommended to
  make sure 'addr' is cache line aliged and 'size` is a multiple of cache line size
  - Start address is calculated as follows, start_addr = floor(addr, CPU_CACHE_LINE)
  - End address is calculated as follows, end_addr = ceil(addr + size, CPU_CACHE_LINE)
  - No error is returned, for unaligned addr and size
- On R5F,
  - Valid values for \ref CacheP_Type for \ref CacheP_enable and \ref CacheP_disable are \ref CacheP_TYPE_L1D, \ref CacheP_TYPE_L1P, \ref CacheP_TYPE_ALL
  - Valid values for \ref CacheP_Type for \ref CacheP_wb, \ref CacheP_wbInv, \ref CacheP_inv, \ref CacheP_wbAll, \ref CacheP_wbInvAll  are \ref CacheP_TYPE_L1D, \ref CacheP_TYPE_ALL
- CPU cache lines sizes for reference
  - R5F: 64B
\cond SOC_AM64X
  - A53: 32B
\endcond
- On R5F, It is recommended to enable both program and data cache in R5F in all applications. In R5F for cache to be enabled MPU needs to be enabled.
- On M4F, the CPU does not support cache so these APIs when used with M4F, will be blank and will have no effect

## Example Usage

Include the below file to access the APIs,
\snippet CacheP_sample.c include

Example usage for cache write back invalidate,
\snippet CacheP_sample.c wbinv

Example usage for cache invalidate,
\snippet CacheP_sample.c inv

## API

\ref KERNEL_DPL_CACHE,
