# Cache for C6x  {#KERNEL_DPL_CACHE_C6X_PAGE}

[TOC]

## Features Supported

- APIs to perform below cache operations
  - set and get cache size for L1P, L1D, L2
  - enable/disable cache for a region of memory (\ref CacheP_setMar)
  - query cache enable/disable status for a region of memory (\ref CacheP_getMar)
- Only valid for below CPUs,
  - c6x
- See also \ref KERNEL_DPL_CACHE_PAGE

## Features NOT Supported

NA

## Important Usage Guidelines

- Refer to the DEVICE datasheet for total available L1P, L1D and L2 size for the c6x CPU.
  - Based on the size set for cache, the size available as SRAM decreases.

## Example Usage

Include the below file to access the APIs,
\snippet CacheP_c6x_sample.c include

Example usage to setup all L1P and L1D as cache and 64K L2 as cache
\snippet CacheP_c6x_sample.c setsize

Example usage to get L1P, L1D, L2 cache size's
\snippet CacheP_c6x_sample.c getsize

Example usage to enable and disable cache for specific regions
\snippet CacheP_c6x_sample.c setmar

## API

\ref KERNEL_DPL_CACHE_C6X, see also \ref KERNEL_DPL_CACHE
