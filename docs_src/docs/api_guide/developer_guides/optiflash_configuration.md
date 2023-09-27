# Configuring OpTI-Flash various IP {#OPTIFLASH_CONFIGURE}

[TOC]

## Introduction

This document gives a basic introduction to configuring different IPs, which is part of OpTI-Flash.

## How to configure RL2?

RL2 is, basically, an L2 cache controller. Highlighting the `controller` keyword here, because, unlike the L1 cache controller, which has internal cache memory, RL2 does not have internal cache memory. RL2 needs to be configured, and a part of On-chip SRAM needs to be assigned for RL2 cache memory.

There are three configurations that are largely needed:
1. Size of the L2 cache
2. Range of memory to cache
3. SRAM memory region to be used as L2 cache memory This SRAM memory region is an L2 remote region.

These configurations can be done from syscfg as follows:

\imageStyle{syscfg.png,width:50%}
\image html syscfg.png

By selecting `Layer2 Cache` from the `OpTI-Flash` drop-down, the above configurations come:

1. Make sure the above module is enabled. This helps in enabling and disabling Layer 2 caches without losing configurations.

2. `External Flash cached region start address` and `External Flash cached region end address` define the range of addresses that need to be cached in L2 Cache.

3. `Size of cache` is the size of L2 cache that is required to be configured.

4. After this, remote regions need to be configured. The remote region should have an address that is L2 SRAM, which in the case of am263px is 0x70000000 and has a size of 3MB. Please check the datasheet for the particular device. In the context of am263px, `0x70000000 <= base address of remote region 0/1/2 <= (0x70000000 + 3MB)`. The total size of the remote region should not be less than the size of the cache; otherwise, this would lead to CPU data/prefetch aborts or undefined aborts. That means `(Length of remote region 0) + (Length of remote region 1) + (Length of remote region 2) <= (Size of Cache)`.

\note It is advised to use `remote region 0` only rather than split the cache bank into 3 regions from the perspective of cache efficiency, or `(Length of remote region 1) = (Length of remote region 2) = 0`.

\note Linker should be modified to make sure that no code or data at compiler time or runtime would overwrite the memory that is reserved for the RL2 remote region.

## How to configure FLC.

FLC is a simplified DMA. To configure FLC, in syscfg, it can be done by clicking to the `Fast Local Copy` nav menu item under OpTI-Flash drop down menu at the bottom.

\imageStyle{flc_syscfg_initial.png,width:50%}
\image html flc_syscfg_initial.png

Use `Add` button to to add FLc instances. For Am263px case, there are a total of 4 FLC instances per R5 core. All info that needs to be given is source address, destination address and size. Make sure that all these fields are 4K aligned.