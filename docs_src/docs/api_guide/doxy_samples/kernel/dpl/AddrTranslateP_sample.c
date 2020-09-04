
//! [include]
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/DebugP.h>
//! [include]

void samples()
{
//! [addr_translate]
    uint64_t systemAddr = 0x029060000ul;
    void *localAddr;

    localAddr = AddrTranslateP_getLocalAddr(systemAddr);

    DebugP_log("System Addr:%x => Local Address:%x\n", systemAddr, localAddr);
//! [addr_translate]
}
