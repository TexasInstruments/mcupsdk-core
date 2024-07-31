# Configuring OptiFlash various IP {#OPTIFLASH_CONFIGURE}

[TOC]

## Introduction

This document gives a basic introduction to configuring different IPs, which is part of OptiFlash.

## How to configure RL2?

RL2 is, basically, an L2 cache controller. Highlighting the `controller` keyword here, because, unlike the L1 cache controller, which has internal cache memory, RL2 does not have internal cache memory. RL2 needs to be configured, and a part of On-chip SRAM needs to be assigned for RL2 cache memory.

There are three configurations that are largely needed:
1. Size of the L2 cache
2. Range of memory to cache
3. SRAM memory region to be used as L2 cache memory This SRAM memory region is an L2 remote region.


Before configuring RL2, 1st requirement is to allocate a part of L2 memory as cache bank. To do this:
- In syscfg, Open `Memory Region`:


\imageStyle{optiflash_rl2_config_memory_region_open.png,width:50%}
\image html optiflash_rl2_config_memory_region_open.png

- Add new Memory region with the following details:


\imageStyle{optiflash_rl2_config_memory_region_new.png,width:50%}
\image html optiflash_rl2_config_memory_region_new.png

Make sure that size of the memory region is more than the size of the cache that is required for the application. Here, since it is required to have 128KB of application, therefore, the size of this memory region `L2_CACHE_BANK` has been set to 128KB.

Now once memory region has been created, RL2 can now be configured and its configuration can be done from syscfg as follows:

\imageStyle{syscfg.png,width:50%}
\image html syscfg.png

By selecting `Layer2 Cache` from the `OptiFlash` drop-down, the above configurations come:

1. Make sure the above module is enabled. This helps in enabling and disabling Layer 2 caches without losing configurations.

2. `External Flash cached region start address` and `External Flash cached region end address` define the range of addresses that need to be cached in L2 Cache.

3. `Size of cache` is the size of L2 cache that is required to be configured.

4. After this, select the memory region where the L2 cache bank should be located. This should be L2 memory (In case of am263px, L2 memory is OCRAM). The memory region that is selected is `L2_CACHE_BANK` which was configured above. Similarly, if some other cache size has to be selected then the size of the cache bank has to be changed correspondingly. 

## How to configure FLC.

FLC is a simplified DMA. To configure FLC, in syscfg, it can be done by clicking to the `Fast Local Copy` nav menu item under OptiFlash drop down menu at the bottom.

\imageStyle{flc_syscfg_initial.png,width:50%}
\image html flc_syscfg_initial.png

Use `Add` button to to add FLC instances. For AM263Px case, there are a total of 4 FLC instances per R5 core. All info that needs to be given is source address, destination address and size. Make sure that all these fields are 4K aligned.

## How to configure RAT.

Region base address translation or RAT can be configured using the following fields.

\imageStyle{rat_syscfg.png,width:50%}
\image html rat_syscfg.png

`Region Size`: Size of RAT region.

`Region Base Address (hex)`: Size aligned base address

`Region Translated Address (hex)`: Size aligned translated address