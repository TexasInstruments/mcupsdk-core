
//! [include]
#include <kernel/dpl/CacheP.h>
//! [include]
 


void samples()
{
//! [wbinv]
    char buf[1024];

    void * addr = buf;
    uint32_t size = sizeof(buf);

    // addr = cache line aligned buffer address
    // size = multiple of cache line aligned size in bytes
    // flush contents of cache to memory so that a DMA or HW peripheral can see the data
    CacheP_wbInv(addr, size, CacheP_TYPE_ALL);

    // send data to DMA or HW peripheral
//! [wbinv]

//! [inv]
    // ...
    // recieve data from DMA or HW peripheral
    // addr = cache line aligned buffer address
    // size = multiple of cache line aligned size in bytes
    // invalidate contents of cache so that a CPU can see the data written by DMA or HW peripheral

    CacheP_inv(addr, size, CacheP_TYPE_ALL);
//! [inv]

}