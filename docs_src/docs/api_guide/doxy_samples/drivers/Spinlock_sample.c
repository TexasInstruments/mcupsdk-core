
#include <stdio.h>
//! [include]
#include <drivers/spinlock.h>
//! [include]
#include <drivers/hw_include/cslr_soc.h>

uint32_t    spinlockBaseAddr = CSL_SPINLOCK0_BASE;
uint32_t    lockNum = 0;

void lock(void)
{
//! [lock]
    int32_t  status;

    /* Spin till lock is acquired */
    while(1U)
    {
        status = Spinlock_lock(spinlockBaseAddr, lockNum);
        if(status == SPINLOCK_LOCK_STATUS_FREE)
        {
            break;  /* Free and taken */
        }
    }

    /*
     * enter critical section
     */
//! [lock]
}

void unlock(void)
{
//! [unlock]
    Spinlock_unlock(spinlockBaseAddr, lockNum);
//! [unlock]
}
